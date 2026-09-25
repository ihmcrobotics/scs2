package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.FlatGroundDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * A {@link FlatGroundDefinition} should reach MuJoCo as its {@code plane} primitive rather than as
 * the 10 km box the definition is built from.
 */
public class MujocoFlatGroundPlaneTest
{
   private static final double BOX_HEIGHT = 0.1;
   private static final double SPAWN_HEIGHT = 0.5;

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

   private static RobotDefinition createBoxRobot()
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("root");
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("box");
      body.setMass(5.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.05, 0.05, 0.05));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(0.2, 0.2, BOX_HEIGHT)));
      rootJoint.setSuccessor(body);

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, SPAWN_HEIGHT));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private Robot createSession()
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame,
                                                                                                rootRegistry,
                                                                                                new MujocoSimulationParameters()));
      session.addTerrainObject(new FlatGroundDefinition());
      session.addRobot(createBoxRobot());
      session.setSessionDTSeconds(1.0e-3);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(5000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   @Test
   public void testFlatGroundCompilesToAPlaneGeom()
   {
      createSession();
      assertTrue(session.getSimulationSessionControls().simulateNow(1), "Simulation reported a failure");

      var model = engine().getDynamicsWorld().getModel();
      boolean foundPlane = false;
      for (int i = 0; i < model.ngeom(); i++)
         foundPlane |= model.geom_type().get(i) == Mujoco.mjGEOM_PLANE;

      assertTrue(foundPlane, "FlatGroundDefinition did not produce a plane geom");
   }

   /** And it has to behave: the plane's surface sits at z = 0, matching the box definition's top face. */
   @Test
   public void testBoxComesToRestOnThePlane()
   {
      Robot robot = createSession();
      assertTrue(session.getSimulationSessionControls().simulateNow(1500), "Simulation reported a failure");

      assertEquals(0.5 * BOX_HEIGHT, robot.getFloatingRootJoint().getJointPose().getZ(), 5.0e-3, "The box did not settle on the ground plane");
   }
}
