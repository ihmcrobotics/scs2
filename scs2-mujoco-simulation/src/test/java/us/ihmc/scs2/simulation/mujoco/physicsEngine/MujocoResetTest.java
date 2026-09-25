package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;

/**
 * Covers the initial keyframe that reset goes through, so a reset returns the robots to the pose
 * they were seeded at rather than to MuJoCo's own qpos0.
 */
public class MujocoResetTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final String ARM_JOINT = "arm";
   private static final double SPAWN_HEIGHT = 0.4;
   private static final double INITIAL_ARM_ANGLE = 0.3;

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

   /** A box that lands on the ground, with an arm on top so there is internal motion too. */
   private static RobotDefinition createRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("box");
      body.setMass(10.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(0.2, 0.2, 0.1)));
      rootJoint.setSuccessor(body);

      RevoluteJointDefinition armJoint = new RevoluteJointDefinition(ARM_JOINT, new Vector3D(0.0, 0.1, 0.0), new Vector3D(0.0, 1.0, 0.0));
      RigidBodyDefinition arm = new RigidBodyDefinition("arm");
      arm.setMass(1.0);
      arm.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      arm.setCenterOfMassOffset(0.0, 0.0, 0.15);
      armJoint.setSuccessor(arm);
      armJoint.setInitialJointState(new OneDoFJointState(INITIAL_ARM_ANGLE));
      body.addChildJoint(armJoint);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, SPAWN_HEIGHT));
      // Sideways velocity so the box is still sliding against friction when the snapshot is taken;
      // a scene that has settled would replay identically no matter what was captured.
      initialState.setVelocity(null, new Vector3D(2.0, 0.0, 0.0));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private static TerrainObjectDefinition createGround()
   {
      TerrainObjectDefinition ground = new TerrainObjectDefinition();
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().set(0.0, 0.0, -0.5);
      ground.addCollisionShapeDefinition(new CollisionShapeDefinition(pose, new Box3DDefinition(10.0, 10.0, 1.0)));
      return ground;
   }

   private Robot createSession()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      // Low sliding friction so the box is still moving several hundred ticks after touchdown; at
      // the default friction it stops almost immediately and the replay comparison becomes vacuous.
      parameters.set_friction_slide(0.15);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addTerrainObject(createGround());
      session.addRobot(createRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, -9.81);
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

   private double[] recordTrajectory(Robot robot, int ticks)
   {
      double[] trajectory = new double[ticks];
      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();
      for (int i = 0; i < ticks; i++)
      {
         simulate(1);
         trajectory[i] = rootJoint.getJointPose().getX();
      }
      return trajectory;
   }

   /** Reset goes through the initial keyframe, so it returns the spawn pose, not MuJoCo's qpos0. */
   @Test
   public void testResetReturnsToTheSeededSpawnState()
   {
      Robot robot = createSession();
      simulate(400);

      assertTrue(Math.abs(robot.getFloatingRootJoint().getJointPose().getZ() - SPAWN_HEIGHT) > 0.1, "The robot did not move away from spawn");

      session.getSimulationSessionControls().resetToInitialState();
      simulate(1);

      assertEquals(SPAWN_HEIGHT, robot.getFloatingRootJoint().getJointPose().getZ(), 5.0e-3, "Reset did not restore the spawn height");
      assertEquals(INITIAL_ARM_ANGLE,
                   ((OneDoFJointBasics) robot.getJoint(ARM_JOINT)).getQ(),
                   5.0e-3,
                   "Reset did not restore the seeded joint angle -- it fell back to qpos0");
   }
}
