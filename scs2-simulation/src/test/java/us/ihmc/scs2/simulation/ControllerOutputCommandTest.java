package us.ihmc.scs2.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
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
import us.ihmc.scs2.simulation.physicsEngine.DoNothingPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * A controller publishes its low-level command into {@code ControllerOutput} as setpoints plus
 * gains, which is what lets an engine that models actuators close the impedance loop itself. The
 * setpoints live in the configuration and velocity, because in a controller's output those are what
 * the controller wants rather than what it measured.
 *
 * <p>{@link DoNothingPhysicsEngine} is the cleanest place to pin that down: it integrates nothing
 * and writes every state the controller published straight onto the joints, so it shows the
 * setpoints arriving without any dynamics in the way.
 */
public class ControllerOutputCommandTest
{
   private static final String JOINT = "pin";

   private SimulationSession session;

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   private Robot createSession()
   {
      RobotDefinition robotDefinition = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      RevoluteJointDefinition joint = new RevoluteJointDefinition(JOINT, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(1.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      joint.setSuccessor(link);
      joint.setInitialJointState(new OneDoFJointState(0.0, 0.0));
      elevator.addChildJoint(joint);
      robotDefinition.setRootBodyDefinition(elevator);

      session = new SimulationSession(DoNothingPhysicsEngine::new);
      session.addRobot(robotDefinition);
      session.initializeBufferSize(100);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   /**
    * The setpoints reach the joints, which is the behaviour a kinematic engine wants and the reason
    * the command reuses the configuration and velocity rather than adding a second pair of fields.
    */
   @Test
   public void testPublishedSetpointsDriveAJointUnderDoNothing()
   {
      double qDesired = 0.35;
      double qdDesired = -0.2;

      Robot robot = createSession();
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(JOINT);
      robot.getControllerManager().addController((Controller) () -> output.setEffortAndCommand(1.5, 0.5, qDesired, qdDesired, 40.0, 2.0));

      assertTrue(session.getSimulationSessionControls().simulateNow(1), "Simulation reported a failure");

      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);
      assertEquals(qDesired, joint.getQ(), 1.0e-12, "The published position setpoint did not reach the joint");
      assertEquals(qdDesired, joint.getQd(), 1.0e-12, "The published velocity setpoint did not reach the joint");
      assertEquals(1.5, joint.getTau(), 1.0e-12, "The published effort did not reach the joint");
   }

   /**
    * The other half of the same contract: a controller that only sets an effort publishes no
    * setpoints, so nothing is written over the joint's configuration.
    */
   @Test
   public void testEffortOnlyControllerLeavesTheConfigurationAlone()
   {
      Robot robot = createSession();
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(JOINT);
      robot.getControllerManager().addController((Controller) () -> output.setEffort(1.5));

      assertTrue(session.getSimulationSessionControls().simulateNow(1), "Simulation reported a failure");

      OneDoFJointBasics joint = (OneDoFJointBasics) robot.getJoint(JOINT);
      assertEquals(0.0, joint.getQ(), 1.0e-12, "An effort-only controller moved the joint");
      assertEquals(0.0, joint.getQd(), 1.0e-12, "An effort-only controller moved the joint");
      assertEquals(1.5, joint.getTau(), 1.0e-12, "The published effort did not reach the joint");
   }
}
