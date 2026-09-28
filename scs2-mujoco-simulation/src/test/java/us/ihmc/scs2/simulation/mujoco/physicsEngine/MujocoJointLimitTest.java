package us.ihmc.scs2.simulation.mujoco.physicsEngine;

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
import us.ihmc.scs2.definition.robot.SphericalJointDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * MuJoCo used to emit no joint limits at all, so a joint driven past its range just kept going.
 * These cover the MJCF {@code range} now emitted from the position limits, and the parameter that
 * turns it off again.
 */
public class MujocoJointLimitTest
{
   private static final double DT = 1.0e-3;
   private static final String JOINT = "hinge";
   private static final double UPPER_LIMIT = 0.5;
   private static final double LOWER_LIMIT = -0.2;

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
      joint.setPositionLimits(LOWER_LIMIT, UPPER_LIMIT);
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(0.05, 0.05, 0.05));
      joint.setSuccessor(link);
      elevator.addChildJoint(joint);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession(boolean enforceJointLimits)
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setEnforceJointLimits(enforceJointLimits);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(10000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   /** Drives the joint with a constant torque, hard enough to run it well past its range. */
   private void addConstantTorque(Robot robot, double tau)
   {
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(JOINT);
      robot.getControllerManager().addController((Controller) () -> output.setEffort(tau));
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   /**
    * A joint type the builder cannot represent must stop the model, not vanish from it. Emitting
    * nothing would weld the body to its parent, and the simulation would run a robot with one fewer
    * degree of freedom than the one it was handed.
    */
   @Test
   public void testUnsupportedJointTypeIsRejected()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SphericalJointDefinition ballJoint = new SphericalJointDefinition("ball");
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      ballJoint.setSuccessor(link);
      elevator.addChildJoint(ballJoint);
      robot.setRootBodyDefinition(elevator);

      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                                rootRegistry,
                                                                                                new MujocoSimulationParameters()));
      session.addRobot(robot);
      session.setSessionDTSeconds(DT);
      session.initializeBufferSize(100);

      RuntimeException thrown = null;
      try
      {
         session.getSimulationSessionControls().simulateNow(1);
      }
      catch (RuntimeException e)
      {
         thrown = e;
      }

      assertTrue(thrown != null, "Expected an unsupported joint type to stop the model being built");
      String message = thrown.getMessage() == null ? "" : thrown.getMessage();
      assertTrue(message.contains("ball") || message.contains("Spherical"),
                 "Expected the failure to name the offending joint, got: " + message);
   }

   @Test
   public void testJointStopsAtItsUpperLimit()
   {
      Robot robot = createSession(true);
      addConstantTorque(robot, 1.0);
      simulate(3000);

      double q = ((OneDoFJointBasics) robot.getJoint(JOINT)).getQ();
      // MuJoCo's default solreflimit is a soft stop, so a little overshoot is expected; what matters
      // is that the joint is held near its range instead of running away.
      assertTrue(q < UPPER_LIMIT + 0.05, "The joint ran past its upper limit; q = " + q);
      assertTrue(q > UPPER_LIMIT - 0.05, "The joint never reached its upper limit; q = " + q);
   }

   @Test
   public void testJointStopsAtItsLowerLimit()
   {
      Robot robot = createSession(true);
      addConstantTorque(robot, -1.0);
      simulate(3000);

      double q = ((OneDoFJointBasics) robot.getJoint(JOINT)).getQ();
      assertTrue(q > LOWER_LIMIT - 0.05, "The joint ran past its lower limit; q = " + q);
      assertTrue(q < LOWER_LIMIT + 0.05, "The joint never reached its lower limit; q = " + q);
   }

   /** The escape hatch: with enforcement off, the old unlimited behavior is back. */
   @Test
   public void testLimitsCanBeDisabled()
   {
      Robot robot = createSession(false);
      addConstantTorque(robot, 1.0);
      simulate(3000);

      double q = ((OneDoFJointBasics) robot.getJoint(JOINT)).getQ();
      assertTrue(q > UPPER_LIMIT + 1.0, "The joint was limited even with enforceJointLimits off; q = " + q);
   }
}
