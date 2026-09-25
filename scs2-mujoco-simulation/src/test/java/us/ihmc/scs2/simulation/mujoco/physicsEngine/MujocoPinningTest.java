package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;

/**
 * Covers {@code SimJointBasics.setPinned(boolean)} under MuJoCo, which used to be a silent no-op
 * there while ContactPointBased and ImpulseBased honored it.
 */
public class MujocoPinningTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final String CHILD_JOINT = "child";
   private static final double SPAWN_HEIGHT = 1.0;

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
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(10.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      rootJoint.setSuccessor(body);

      // A heavy arm hanging off a hinge, so a pinned root has real load to hold.
      RevoluteJointDefinition childJoint = new RevoluteJointDefinition(CHILD_JOINT, new Vector3D(0.0, 0.2, 0.0), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(5.0);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      link.setCenterOfMassOffset(0.0, 0.0, -0.3);
      childJoint.setSuccessor(link);
      body.addChildJoint(childJoint);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, SPAWN_HEIGHT));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession()
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, new MujocoSimulationParameters()));
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(20000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   /** A pinned floating root must hold its height under gravity, and fall again once unpinned. */
   @Test
   public void testPinnedRootHoldsAndReleases()
   {
      Robot robot = createSession();
      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();
      rootJoint.setPinned(true);

      simulate(1000); // 1 s of gravity on a 15 kg robot.
      assertEquals(SPAWN_HEIGHT, rootJoint.getJointPose().getZ(), 1.0e-3, "The pinned root did not hold its height");

      rootJoint.setPinned(false);
      simulate(500);
      assertTrue(rootJoint.getJointPose().getZ() < SPAWN_HEIGHT - 0.5 * 9.81 * 0.5 * 0.5 * 0.5,
                 "The root did not fall after being unpinned; z = " + rootJoint.getJointPose().getZ());
   }

   /**
    * Pinning after moving is the recipe the obstacle-course tests use: set the pose through SCS2,
    * then pin, and the robot stays where it was put.
    */
   @Test
   public void testRootHoldsThePoseItWasMovedTo()
   {
      Robot robot = createSession();
      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();
      simulate(10);

      Point3D target = new Point3D(0.3, -0.2, 1.5);
      rootJoint.getJointPose().getPosition().set(target);
      rootJoint.getJointTwist().setToZero();
      robot.updateFrames();
      rootJoint.setPinned(true);

      simulate(1000);

      // A constraint-based pin is compliant by design: the hanging arm's moment is reacted through
      // the solver, so the root settles a fraction of a millimetre off the commanded pose rather
      // than being teleported back onto it exactly.
      assertEquals(target.getX(), rootJoint.getJointPose().getX(), 5.0e-3);
      assertEquals(target.getY(), rootJoint.getJointPose().getY(), 5.0e-3);
      assertEquals(target.getZ(), rootJoint.getJointPose().getZ(), 5.0e-3);

      // What must not happen is the pin slowly walking away from its target.
      Point3D settled = new Point3D(rootJoint.getJointPose().getPosition());
      simulate(1000);
      assertEquals(settled.getX(), rootJoint.getJointPose().getX(), 1.0e-5, "The pin drifted in x");
      assertEquals(settled.getY(), rootJoint.getJointPose().getY(), 1.0e-5, "The pin drifted in y");
      assertEquals(settled.getZ(), rootJoint.getJointPose().getZ(), 1.0e-5, "The pin drifted in z");
   }

   /** A pinned 1-DoF joint must hold its angle against the gravity moment on the link. */
   @Test
   public void testPinnedOneDoFJointHoldsItsAngle()
   {
      Robot robot = createSession();
      robot.getFloatingRootJoint().setPinned(true);

      SimJointBasics childJoint = robot.getJoint(CHILD_JOINT);
      OneDoFJointBasics childOneDoF = (OneDoFJointBasics) childJoint;
      simulate(10);

      childOneDoF.setQ(0.4);
      childOneDoF.setQd(0.0);
      robot.updateFrames();
      childJoint.setPinned(true);

      simulate(1000);
      assertEquals(0.4, childOneDoF.getQ(), 1.0e-3, "The pinned hinge did not hold its angle");

      childJoint.setPinned(false);
      simulate(500);
      assertTrue(Math.abs(childOneDoF.getQ() - 0.4) > 0.05,
                 "The hinge did not swing after being unpinned; q = " + childOneDoF.getQ());
   }
}
