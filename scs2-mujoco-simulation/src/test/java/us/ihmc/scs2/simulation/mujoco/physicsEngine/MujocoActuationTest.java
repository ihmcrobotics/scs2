package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Covers the joint servo: that handing MuJoCo the setpoints and gains reproduces the control law
 * SCS2 has always applied, that doing so makes the damping term integrable implicitly (which is the
 * reason for it), that the torque decomposition matches the force MuJoCo reports, and that a plain
 * effort-setting controller still drives the joint.
 */
public class MujocoActuationTest
{
   private static final String JOINT = "pin";
   private static final double LINK_INERTIA = 0.01;

   private SimulationSession session;

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   /**
    * A single hinge joint attached straight to the world, with the link's mass at the joint origin
    * so gravity applies no moment about the axis and the joint's response is purely what the
    * actuation produces.
    */
   private static RobotDefinition createRobot(double initialQ, double initialQd)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");

      RevoluteJointDefinition joint = new RevoluteJointDefinition(JOINT, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(LINK_INERTIA, LINK_INERTIA, LINK_INERTIA));
      joint.setSuccessor(link);
      joint.setInitialJointState(new OneDoFJointState(initialQ, initialQd));
      elevator.addChildJoint(joint);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession(double dt, double initialQ, double initialQd)
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot(initialQ, initialQd));
      session.setSessionDTSeconds(dt);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(10000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   /**
    * Applies the same law SCS2OutputWriter applies, on the SCS2 side, so a TORQUE_PASSTHROUGH run
    * can be compared against a JOINT_SERVO run driven by the same numbers.
    */
   private static final class Scs2SidePDController implements Controller
   {
      private final OneDoFJointBasics measured;
      private final OneDoFJointStateBasics output;
      private final double feedforwardTorque, desiredPosition, desiredVelocity, stiffness, damping;

      private Scs2SidePDController(OneDoFJointBasics measured,
                                   OneDoFJointStateBasics output,
                                   double feedforwardTorque,
                                   double desiredPosition,
                                   double desiredVelocity,
                                   double stiffness,
                                   double damping)
      {
         this.measured = measured;
         this.output = output;
         this.feedforwardTorque = feedforwardTorque;
         this.desiredPosition = desiredPosition;
         this.desiredVelocity = desiredVelocity;
         this.stiffness = stiffness;
         this.damping = damping;
      }

      @Override
      public void doControl()
      {
         output.setEffort(feedforwardTorque + stiffness * (desiredPosition - measured.getQ()) + damping * (desiredVelocity - measured.getQd()));
      }
   }

   private void addScs2SidePD(Robot robot, double tauFF, double qDesired, double qdDesired, double kp, double kd)
   {
      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(JOINT);
      robot.getControllerManager().addController(new Scs2SidePDController(joint, output, tauFF, qDesired, qdDesired, kp, kd));
   }

   private void addJointServoCommand(Robot robot, double tauFF, double qDesired, double qdDesired, double kp, double kd)
   {
      robot.getControllerManager().addController(new Controller()
      {
         @Override
         public void doControl()
         {
            MujocoJointActuation actuation = engine().getJointActuation(JOINT);
            if (actuation != null)
               actuation.setCommand(tauFF, qDesired, qdDesired, kp, kd);
         }
      });
   }

   /**
    * With the damping term switched off and a timestep both schemes are stable at, letting MuJoCo
    * close the loop must reproduce what SCS2 computes itself and sends as effort. This is the check
    * on the gain and bias arithmetic: a swapped gainprm/biasprm slot shows up here immediately.
    */
   @Test
   public void testServoReproducesAnEquivalentJavaSidePD()
   {
      double dt = 1.0e-4;
      double tauFF = 0.3;
      double qDesired = 0.4;
      double kp = 5.0;
      int ticks = 2000;

      Robot javaSideRobot = createSession(dt, 0.0, 0.0);
      addScs2SidePD(javaSideRobot, tauFF, qDesired, 0.0, kp, 0.0);
      simulate(ticks);
      double javaSideQ = ((OneDoFJointBasics) javaSideRobot.getJoint(JOINT)).getQ();
      double javaSideQd = ((OneDoFJointBasics) javaSideRobot.getJoint(JOINT)).getQd();
      shutdown();

      Robot servoRobot = createSession(dt, 0.0, 0.0);
      addJointServoCommand(servoRobot, tauFF, qDesired, 0.0, kp, 0.0);
      simulate(ticks);
      double servoQ = ((OneDoFJointBasics) servoRobot.getJoint(JOINT)).getQ();
      double servoQd = ((OneDoFJointBasics) servoRobot.getJoint(JOINT)).getQd();

      assertEquals(javaSideQ, servoQ, 1.0e-6, "The servo did not reproduce the Java-side PD position");
      assertEquals(javaSideQd, servoQd, 1.0e-4, "The servo did not reproduce the Java-side PD velocity");
      // Sanity: the setpoint was actually tracked, so the comparison above is not between two zeros.
      assertTrue(Math.abs(servoQ - qDesired) < 0.2, "The servo never approached the setpoint; q = " + servoQ);
   }

   /**
    * The reason the mode exists. A damping term applied as a held-constant external torque is
    * explicit, so it goes unstable once {@code kd * dt} is large compared to the joint's inertia.
    * Handed to MuJoCo as an affine actuator bias it is folded into the implicit velocity update by
    * the IMPLICITFAST integrator SCS2 defaults to, and stays stable at the same timestep.
    */
   @Test
   public void testDampingIsIntegratedImplicitlyUnderJointServo()
   {
      // Explicit Euler on a damping torque multiplies the velocity by (1 - kd * dt / I) each step,
      // so it is stable only while kd * dt / I < 2. Here dt / I = 1, and kd = 2.1 puts the explicit
      // scheme just over the edge at a factor of -1.1 per step: it grows steadily over a short run
      // without reaching the magnitude at which MuJoCo raises mjWARN_BADQACC and resets the state
      // (which would silently hand the test a tidy qd of zero). Do not crank these numbers.
      double dt = 1.0e-2;
      double kd = 2.1;
      double initialQd = 1.0;
      int ticks = 20;

      Robot explicitRobot = createSession(dt, 0.0, initialQd);
      addScs2SidePD(explicitRobot, 0.0, 0.0, 0.0, 0.0, kd);
      simulate(ticks);
      double explicitQd = ((OneDoFJointBasics) explicitRobot.getJoint(JOINT)).getQd();
      shutdown();

      Robot servoRobot = createSession(dt, 0.0, initialQd);
      addJointServoCommand(servoRobot, 0.0, 0.0, 0.0, 0.0, kd);
      simulate(ticks);
      double servoQd = ((OneDoFJointBasics) servoRobot.getJoint(JOINT)).getQd();

      assertTrue(Math.abs(explicitQd) > 2.0 * initialQd,
                 "Expected the explicit damping torque to diverge at this timestep, but qd = " + explicitQd);
      assertTrue(Math.abs(servoQd) < 0.5 * initialQd,
                 "Expected the implicitly integrated damping to settle the joint, but qd = " + servoQd);
   }

   /**
    * The decomposition the SCS2 pipeline publishes has to survive the move into MuJoCo: each
    * actuator's force is reported separately and the three add up to what was applied.
    */
   @Test
   public void testActuatorForcesDecomposeTheAppliedTorque()
   {
      double dt = 1.0e-3;
      double tauFF = 0.2;
      double qDesired = 0.3;
      double qdDesired = 0.1;
      double kp = 4.0;
      double kd = 0.05;

      Robot robot = createSession(dt, 0.0, 0.0);
      addJointServoCommand(robot, tauFF, qDesired, qdDesired, kp, kd);
      simulate(49);

      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);
      // MuJoCo evaluates the actuator at the start of a step, so the terms belong to this state, not
      // to the state the step lands in. The engine captures the same values when it pushes.
      double qBeforeStep = joint.getQ();
      double qdBeforeStep = joint.getQd();
      simulate(1);

      MujocoJointActuation actuation = engine().getJointActuation(JOINT);
      assertNotNull(actuation, "No actuation block was created for the joint");

      assertEquals(tauFF, actuation.getControllerTau(), 1.0e-9);
      assertEquals(kp * (qDesired - qBeforeStep), actuation.getPositionTau(), 1.0e-9);
      assertEquals(kd * (qdDesired - qdBeforeStep), actuation.getVelocityTau(), 1.0e-9);

      // The decomposition is computed rather than read back, so pin it to the force MuJoCo reports.
      double sum = actuation.getControllerTau() + actuation.getPositionTau() + actuation.getVelocityTau();
      assertEquals(sum, actuation.getAppliedTau(), 1.0e-9, "The decomposition does not sum to the force MuJoCo applied");
      // pullActuationFromMujoco writes the applied total onto the SCS2 joint.
      assertEquals(sum, joint.getTau(), 1.0e-9, "The joint's tau does not match the applied torque");
      // Guard against all three terms being trivially zero.
      assertTrue(Math.abs(sum) > 1.0e-3, "The actuator applied nothing; sum = " + sum);
   }

   /**
    * The fallback that lets the servo be the only path. An ordinary SCS2 controller sets effort and
    * knows nothing about actuators; the engine turns that into a pure feedforward command, so the
    * joint sees the same torque it would have as an applied force.
    */
   @Test
   public void testEffortOnlyControllerStillDrivesTheJoint()
   {
      Robot robot = createSession(1.0e-3, 0.0, 0.0);
      addScs2SidePD(robot, 0.5, 0.0, 0.0, 0.0, 0.0); // Effort only: no setpoints, no gains.
      simulate(200);

      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);
      assertTrue(joint.getQd() > 0.0, "A plain effort-setting controller did not drive the joint");
      // 0.5 N*m on 0.01 kg*m^2 for 0.2 s.
      assertEquals(0.5 / LINK_INERTIA * 0.2, joint.getQd(), 1.0e-2, "The effort did not arrive intact");

      MujocoJointActuation actuation = engine().getJointActuation(JOINT);
      assertNotNull(actuation, "Every 1-DoF joint should have an actuator now");
      assertEquals(0.5, actuation.getControllerTau(), 1.0e-9, "The effort should appear as the feedforward term");
      assertEquals(0.0, actuation.getPositionTau(), 1.0e-12);
      assertEquals(0.0, actuation.getVelocityTau(), 1.0e-12);
   }
}
