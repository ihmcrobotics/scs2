package us.ihmc.scs2.simulation.physicsEngine.impulseBased;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * A box resting on flat ground is the case this engine cannot resolve unaided: it computes a single contact point per
 * pair of shapes, and two flat faces do not define one, so the box sinks. The engine rounds the robots' sharp shapes
 * to avoid that, and these tests pin that behavior down.
 */
public class ImpulseBasedShapeRoundingTest
{
   private static final double DT = 1.0e-3;
   private static final String ROOT_JOINT = "root";
   private static final Vector3D BOX_SIZE = new Vector3D(0.3, 0.2, 0.1);
   /** Sole of the box when it rests on the ground, i.e. half its height. */
   private static final double REST_HEIGHT = 0.5 * BOX_SIZE.getZ();

   private SimulationSession session;

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   private static RobotDefinition createBoxRobot()
   {
      RobotDefinition robot = new RobotDefinition("box");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(ROOT_JOINT);
      elevator.addChildJoint(rootJoint);

      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(10.0);
      body.setMomentOfInertia(new MomentOfInertiaDefinition(0.1, 0.1, 0.1));
      body.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(BOX_SIZE)));
      rootJoint.setSuccessor(body);

      SixDoFJointState initialState = new SixDoFJointState();
      // A millimetre of clearance, so the box settles rather than starting in contact.
      initialState.setConfiguration(null, new Point3D(0.0, 0.0, REST_HEIGHT + 1.0e-3));
      rootJoint.setInitialJointState(initialState);

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   private static TerrainObjectDefinition createGround()
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().setZ(-0.5);
      TerrainObjectDefinition ground = new TerrainObjectDefinition();
      ground.addCollisionShapeDefinition(new CollisionShapeDefinition(pose, new Box3DDefinition(10.0, 10.0, 1.0)));
      return ground;
   }

   /** @return how far the box ended up below its resting height, in metres. */
   private double simulateAndMeasureSinking(double roundingMinimumMargin, double roundingMaximumMargin)
   {
      session = new SimulationSession((inertialFrame, rootRegistry) ->
      {
         ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(inertialFrame, rootRegistry);
         physicsEngine.setCollisionShapeRoundingMargins(roundingMinimumMargin, roundingMaximumMargin);
         return physicsEngine;
      });
      session.setSessionDTSeconds(DT);
      session.addTerrainObject(createGround());
      Robot robot = (Robot) session.addRobot(createBoxRobot());
      session.initializeBufferSize(5000);

      assertTrue(session.getSimulationSessionControls().simulateNow(1000), "Simulation reported a failure");

      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getJoint(ROOT_JOINT);
      return REST_HEIGHT - rootJoint.getJointPose().getZ();
   }

   @Test
   public void testBoxRestsOnGroundWithRounding()
   {
      double sinking = simulateAndMeasureSinking(1.0e-5, 4.0e-4);
      assertTrue(Math.abs(sinking) < 1.0e-3, "The box did not rest on the ground, it ended up " + sinking + " m below its resting height");
   }

   @Test
   public void testBoxSinksWithoutRounding()
   {
      double sinking = simulateAndMeasureSinking(0.0, 0.0);
      assertTrue(sinking > 1.0e-2, "Expected the sharp box to sink, but it ended up " + sinking + " m below its resting height");
   }
}
