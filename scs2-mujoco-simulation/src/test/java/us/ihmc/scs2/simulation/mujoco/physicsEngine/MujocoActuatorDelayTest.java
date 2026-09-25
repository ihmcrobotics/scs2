package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
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
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoActuationMode;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Covers command latency on the JOINT_SERVO actuators: the lag a real drive has between being sent
 * a setpoint and producing torque.
 */
public class MujocoActuatorDelayTest
{
   private static final double DT = 1.0e-3;
   private static final String JOINT = "hinge";
   private static final double DELAY = 0.02;

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

   private static RobotDefinition createRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");

      RevoluteJointDefinition joint = new RevoluteJointDefinition(JOINT, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      joint.setSuccessor(link);
      elevator.addChildJoint(joint);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession(double delay)
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setActuationMode(MujocoActuationMode.JOINT_SERVO);
      parameters.setActuatorDelay(delay);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(5000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   private void addConstantTorqueCommand(Robot robot, double tau)
   {
      robot.getControllerManager().addController((Controller) () ->
      {
         MujocoJointActuation actuation = engine().getJointActuation(JOINT);
         if (actuation != null)
            actuation.setCommand(tau, 0.0, 0.0, 0.0, 0.0);
      });
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   /** A step command must not move the joint until the delay has elapsed. */
   @Test
   public void testDelayedCommandArrivesLate()
   {
      Robot robot = createSession(DELAY);
      addConstantTorqueCommand(robot, 0.5);
      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);

      // Half a delay in, the command has not landed yet.
      simulate((int) (0.5 * DELAY / DT));
      double qdMidDelay = joint.getQd();

      // Well past it, the joint is accelerating.
      simulate((int) (3.0 * DELAY / DT));
      double qdAfterDelay = joint.getQd();

      assertTrue(Math.abs(qdMidDelay) < 1.0e-9, "The joint moved before the delay elapsed; qd = " + qdMidDelay);
      assertTrue(qdAfterDelay > 0.1, "The joint never responded after the delay; qd = " + qdAfterDelay);
   }

   /** Same command with no delay moves the joint immediately, and ends up further along. */
   @Test
   public void testDelayLagsAnUndelayedRun()
   {
      Robot delayed = createSession(DELAY);
      addConstantTorqueCommand(delayed, 0.5);
      simulate(200);
      double delayedQ = ((OneDoFJointBasics) delayed.getJoint(JOINT)).getQ();
      shutdown();

      Robot immediate = createSession(0.0);
      addConstantTorqueCommand(immediate, 0.5);
      simulate(200);
      double immediateQ = ((OneDoFJointBasics) immediate.getJoint(JOINT)).getQ();

      assertTrue(immediateQ > delayedQ, "The delayed run should lag: delayed q = " + delayedQ + ", immediate q = " + immediateQ);
      assertTrue(delayedQ > 0.0, "The delayed run never moved at all");
   }

   /** MuJoCo needs at least two history samples to hold a delay; guard the caller rather than fail at compile. */
   @Test
   public void testTooFewHistorySamplesIsRejected()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      assertThrows(IllegalArgumentException.class, () -> parameters.setActuatorDelaySamples(1));
      assertEquals(16, parameters.getActuatorDelaySamples(), "Unexpected default sample count");
   }
}
