package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoContactProperties;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;

/**
 * Covers per-body contact properties: a named class, emitted as a nested MuJoCo
 * {@code <default class="...">}, changing the contact behaviour of just the bodies assigned to it.
 */
public class MujocoContactClassTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final String SLIDER_CLASS = "slippery";

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

   /** A box dropped just above the ground, given a sideways shove. How far it slides tells us the friction. */
   private static RobotDefinition createBoxRobot()
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

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, 0.0501));
      initialState.setVelocity(null, new Vector3D(1.0, 0.0, 0.0));
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

   private Robot createSession(MujocoSimulationParameters parameters)
   {
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addTerrainObject(createGround());
      session.addRobot(createBoxRobot());
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, -9.81);
      session.initializeBufferSize(5000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private double slideDistance(Robot robot)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(1500), "Simulation reported a failure");
      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();
      return rootJoint.getJointPose().getX();
   }

   /**
    * The point of the feature: one body's contact properties changed without touching the model-wide
    * values. The class lowers sliding friction and takes priority so its number wins over the
    * ground's rather than being combined with it.
    */
   @Test
   public void testContactClassChangesOnlyTheAssignedBody()
   {
      MujocoSimulationParameters baseline = new MujocoSimulationParameters();
      double slideWithModelFriction = slideDistance(createSession(baseline));
      shutdown();

      MujocoSimulationParameters slippery = new MujocoSimulationParameters();
      MujocoContactProperties properties = new MujocoContactProperties();
      properties.setFrictionSlide(0.02);
      properties.setPriority(2);
      slippery.addContactClass(SLIDER_CLASS, properties);
      slippery.assignContactClass("box", SLIDER_CLASS);
      double slideWithClassFriction = slideDistance(createSession(slippery));

      assertTrue(slideWithClassFriction > 3.0 * slideWithModelFriction,
                 "The contact class did not reduce friction; slid " + slideWithClassFriction + " m vs " + slideWithModelFriction + " m");
   }

   /** A body that was never assigned a class must be unaffected by one existing. */
   @Test
   public void testUnassignedBodiesKeepTheModelWideProperties()
   {
      MujocoSimulationParameters baseline = new MujocoSimulationParameters();
      double slideWithoutClass = slideDistance(createSession(baseline));
      shutdown();

      MujocoSimulationParameters withUnusedClass = new MujocoSimulationParameters();
      MujocoContactProperties properties = new MujocoContactProperties();
      properties.setFrictionSlide(0.02);
      properties.setPriority(2);
      withUnusedClass.addContactClass(SLIDER_CLASS, properties);
      // Deliberately assigned to a body that does not exist, so nothing should pick it up.
      withUnusedClass.assignContactClass("some_other_body", SLIDER_CLASS);
      double slideWithUnusedClass = slideDistance(createSession(withUnusedClass));

      assertEquals(slideWithoutClass, slideWithUnusedClass, 1.0e-9, "An unassigned contact class changed the simulation");
   }

   /** The builder owns "robot" and "terrain"; taking either would silently break collision filtering. */
   @Test
   public void testReservedClassNamesAreRejected()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      MujocoContactProperties properties = new MujocoContactProperties();
      properties.setFrictionSlide(0.5);

      assertThrows(IllegalArgumentException.class, () -> parameters.addContactClass("robot", properties));
      assertThrows(IllegalArgumentException.class, () -> parameters.addContactClass("terrain", properties));
   }
}
