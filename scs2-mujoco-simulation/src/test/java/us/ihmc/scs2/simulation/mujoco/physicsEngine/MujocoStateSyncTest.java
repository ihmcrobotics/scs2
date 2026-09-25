package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
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

/**
 * Checks that the SCS2 joints are the state of record under MuJoCo, as with the other SCS2 engines: joint state
 * edited through SCS2 between steps must be what MuJoCo continues from, and the pull/push round trip done on every
 * step must not perturb an untouched simulation.
 */
public class MujocoStateSyncTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final String CHILD_JOINT = "child";

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

   /** A floating body with isotropic inertia, optionally carrying a child link on a revolute joint. No collisions. */
   private static RobotDefinition createRobot(boolean withChild)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(2.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      rootJoint.setSuccessor(body);

      if (withChild)
      {
         RevoluteJointDefinition childJoint = new RevoluteJointDefinition(CHILD_JOINT, new Vector3D(0.0, 0.2, 0.0), new Vector3D(0.0, 1.0, 0.0));
         RigidBodyDefinition link = new RigidBodyDefinition("link");
         link.setMass(0.5);
         link.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
         link.setCenterOfMassOffset(0.0, 0.0, -0.1);
         childJoint.setSuccessor(link);
         body.addChildJoint(childJoint);
      }

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, 1.0));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession(boolean withChild, double gravityZ)
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, new MujocoSimulationParameters()));
      session.addRobot(createRobot(withChild));
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, gravityZ);
      session.initializeBufferSize(10000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   @Test
   public void testTeleportThroughScs2JointsIsHonored()
   {
      Robot robot = createSession(true, 0.0);
      simulate(10); // Compiles the MuJoCo model and runs a few steps.

      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getJoint(ROOT_JOINT);
      OneDoFJointBasics childJoint = (OneDoFJointBasics) robot.getJoint(CHILD_JOINT);

      Point3D teleportPosition = new Point3D(1.0, 2.0, 3.0);
      Quaternion teleportOrientation = new Quaternion(Math.toRadians(90.0), 0.0, 0.0);
      rootJoint.getJointPose().getPosition().set(teleportPosition);
      rootJoint.getJointPose().getOrientation().set(teleportOrientation);
      rootJoint.getJointTwist().setToZero();
      childJoint.setQ(0.5);
      childJoint.setQd(0.0);
      robot.updateFrames();

      // No gravity, no velocity, no torques and no contacts: MuJoCo must continue exactly from the edited state.
      simulate(1);

      assertEquals(teleportPosition.getX(), rootJoint.getJointPose().getX(), 1.0e-9);
      assertEquals(teleportPosition.getY(), rootJoint.getJointPose().getY(), 1.0e-9);
      assertEquals(teleportPosition.getZ(), rootJoint.getJointPose().getZ(), 1.0e-9);
      assertEquals(Math.toRadians(90.0), rootJoint.getJointPose().getOrientation().getYaw(), 1.0e-9);
      assertEquals(0.5, childJoint.getQ(), 1.0e-9);
   }

   /**
    * The everyday case: reach in and move a joint on a robot that is not pinned, under gravity,
    * mid-simulation. MuJoCo must continue from the joint angle that was set, and then keep
    * simulating from it rather than either ignoring it or holding it there.
    */
   @Test
   public void testJointEditUnderGravityIsHonoredThenSimulatedFrom()
   {
      Robot robot = createSession(true, -9.81);
      // Hold the base so the arm has a fixed reference to swing against. In free fall the whole
      // system accelerates together, no relative moment reaches the joint, and it would correctly
      // just sit wherever it was put -- which would not distinguish live state from a hold.
      robot.getFloatingRootJoint().setPinned(true);
      simulate(200); // Let it move under gravity so the edit is against a non-trivial state.

      OneDoFJointBasics childJoint = (OneDoFJointBasics) robot.getJoint(CHILD_JOINT);
      double qBeforeEdit = childJoint.getQ();
      double editedQ = qBeforeEdit + 0.5;
      childJoint.setQ(editedQ);
      childJoint.setQd(0.0);
      robot.updateFrames();

      // One tick cannot move the joint far, so this is the edit landing rather than being ignored.
      simulate(1);
      assertEquals(editedQ, childJoint.getQ(), 1.0e-3, "The joint edit was ignored");

      // And it is live state, not a hold: gravity keeps acting from the edited angle.
      simulate(200);
      assertTrue(Math.abs(childJoint.getQ() - editedQ) > 1.0e-3,
                 "The joint was pinned to the edited value rather than simulated from it");
      assertTrue(Math.abs(childJoint.getQ() - qBeforeEdit) > 1.0e-3,
                 "The joint snapped back to its pre-edit value");
   }

   @Test
   public void testUntouchedRoundTripDoesNotPerturbFreeFlight()
   {
      double g = -9.81;
      Robot robot = createSession(false, g);
      simulate(1); // Compile.

      // Tumbling ballistic flight from a non-trivial orientation, so the per-step world/body frame conversions of the
      // pulled and pushed velocities are exercised in both directions.
      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getJoint(ROOT_JOINT);
      Point3D p0 = new Point3D(0.3, -0.2, 5.0);
      Vector3D v0World = new Vector3D(0.4, -0.3, 1.5);
      Quaternion orientation = new Quaternion(0.7, 0.3, -0.4);
      rootJoint.getJointPose().getPosition().set(p0);
      rootJoint.getJointPose().getOrientation().set(orientation);
      Vector3D v0Body = new Vector3D(v0World);
      orientation.inverseTransform(v0Body);
      rootJoint.getJointTwist().getLinearPart().set(v0Body);
      rootJoint.getJointTwist().getAngularPart().set(0.8, -0.5, 1.2); // Isotropic inertia: stays constant.
      robot.updateFrames();

      int ticks = 500;
      simulate(ticks);
      double t = ticks * DT;

      // The integrator's first-order position error is |g| * DT * t / 2 = 2.5 mm here; a wrong frame conversion in
      // the round trip would show up as metres.
      Point3D expectedPosition = new Point3D(p0.getX() + v0World.getX() * t, p0.getY() + v0World.getY() * t, p0.getZ() + v0World.getZ() * t + 0.5 * g * t * t);
      assertTrue(expectedPosition.distance(rootJoint.getJointPose().getPosition()) < 5.0e-3,
                 "Expected " + expectedPosition + " but was " + rootJoint.getJointPose().getPosition());

      Vector3D velocityWorld = new Vector3D(rootJoint.getJointTwist().getLinearPart());
      rootJoint.getJointPose().getOrientation().transform(velocityWorld);
      Vector3D expectedVelocity = new Vector3D(v0World.getX(), v0World.getY(), v0World.getZ() + g * t);
      assertTrue(expectedVelocity.epsilonEquals(velocityWorld, 1.0e-6), "Expected " + expectedVelocity + " but was " + velocityWorld);
      assertTrue(new Vector3D(0.8, -0.5, 1.2).epsilonEquals(rootJoint.getJointTwist().getAngularPart(), 1.0e-6),
                 "Angular velocity drifted: " + rootJoint.getJointTwist().getAngularPart());
   }

   @Test
   public void testVelocityEditThroughScs2JointsIsHonored()
   {
      Robot robot = createSession(false, 0.0);
      simulate(1); // Compile.

      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getJoint(ROOT_JOINT);
      double yaw0 = Math.toRadians(90.0);
      rootJoint.getJointPose().getPosition().set(0.0, 0.0, 1.0);
      rootJoint.getJointPose().getOrientation().setYawPitchRoll(yaw0, 0.0, 0.0);
      // Body-frame forward velocity: with the body yawed 90 degrees this is +y in world.
      rootJoint.getJointTwist().getLinearPart().set(1.0, 0.0, 0.0);
      rootJoint.getJointTwist().getAngularPart().setToZero();
      robot.updateFrames();

      int ticks = 100;
      simulate(ticks);
      double t = ticks * DT;

      assertEquals(0.0, rootJoint.getJointPose().getX(), 1.0e-9);
      assertEquals(t, rootJoint.getJointPose().getY(), 1.0e-9);
      assertEquals(1.0, rootJoint.getJointPose().getZ(), 1.0e-9);
      assertEquals(0.0, EuclidCoreTools.trimAngleMinusPiToPi(rootJoint.getJointPose().getOrientation().getYaw() - yaw0), 1.0e-9);
   }
}
