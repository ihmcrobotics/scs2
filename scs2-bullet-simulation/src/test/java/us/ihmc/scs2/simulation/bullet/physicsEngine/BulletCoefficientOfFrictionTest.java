package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.euclid.YoPoint3D;

public class BulletCoefficientOfFrictionTest
{
   private static final double EPSILON = 0.01;
   private static final boolean VISUALIZE = false;
   private static final int ITERATIONS = 100;
   // A coefficient of friction of 0.7 means that the max angle for the reaction force is 34.95 degrees (angle = atan(coeffOfFriction))
   private static final double GROUND_ANGLE_FOR_MANUAL_TEST = 34.0;
   private static final double MAX_GROUND_ANGLE_FOR_RANDOM = 50.0;

   @Test
   public void testCoefficientOfFriction() throws Throwable
   {
      conductSlideTest(0.5, 0.5);
      conductSlideTest(1.0, 0.5);
      conductSlideTest(1.0, 0.7);
      conductSlideTest(0.7, 1.0);
      conductSlideTest(1.0, 1.0);
   }

   private static void conductSlideTest(double boxFriction, double groundFriction) throws Throwable
   {
      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);

      double angleOfGroundDegrees = GROUND_ANGLE_FOR_MANUAL_TEST;
      double friction = boxFriction;
      double resultantCoeficientOfFriction = boxFriction * groundFriction;
      double groundHeight = 0.01;
      double groundPitch = Math.toRadians(angleOfGroundDegrees);
      boolean shouldSlide = resultantCoeficientOfFriction <= Math.abs(Math.tan(groundPitch));
      double slopeThreshold = Math.atan(resultantCoeficientOfFriction);
      String name = "box";
      double dt = 0.001;

      RobotDefinition boxRobot = newBoxRobot(name, boxSize, 150.0, 0.8, ColorDefinitions.DarkCyan());
      SixDoFJointState initialState = new SixDoFJointState();
      Point3D initialPosition = new Point3D(0.0, 0.0, calculateZ(groundPitch));
      initialState.setConfiguration(new YawPitchRoll(0.0, groundPitch, 0.0), initialPosition);
      initialState.setVelocity(new Vector3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialState);

      boxRobot.getRigidBodyDefinition("boxRigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize)));

      GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, groundHeight);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.appendPitchRotation(groundPitch);
      terrainPose.appendTranslation(0.0, 0.0, 0.0);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.Lavender())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      BulletMultiBodyJointParameters bulletMultiBodyJointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      bulletMultiBodyJointParameters.setJointFriction(friction);

      SimulationSession simulationSession
            = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(BulletMultiBodyParameters.defaultBulletMultiBodyParameters(),
                                                                                             bulletMultiBodyJointParameters));
      simulationSession.addRobot(boxRobot);
      simulationSession.addTerrainObject(terrain); // The terrain friction is probably 1.0
      simulationSession.setSessionDTSeconds(dt);

      if (simulationSession.getPhysicsEngine() instanceof BulletPhysicsEngine)
      {
         BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
         bulletPhysicsEngine.getTerrainObjects().get(0).getBtRigidBody().setFriction(groundFriction);
      }

      SessionVisualizerControls visualizerControls = null;

      if (VISUALIZE)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         visualizerControls.waitUntilVisualizerFullyUp();
      }

      YoPoint3D expectedPosition = new YoPoint3D("expectedSpherePosition", simulationSession.getRootRegistry());
      SimFloatingRootJoint floatingRootJoint = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(0).getJoint(name);

      MutableObject<Throwable> caughtException = new MutableObject<>(null);
      simulationSession.addRunThrowableListener(caughtException::setValue);
      final MutableBoolean printed = new MutableBoolean(false);
      simulationSession.addAfterPhysicsCallback(time ->
      {
         expectedPosition.set(initialPosition);
         if (shouldSlide)
         {
            if (time > 0.5)
            {
               Assertions.assertNotEquals(expectedPosition.getZ(), floatingRootJoint.getJointPose().getPosition().getZ(), "Should slide.");
            }
         }
         else
         {
            EuclidCoreTestTools.assertEquals("Should not slide.", expectedPosition, floatingRootJoint.getJointPose().getPosition(), EPSILON);
         }

         if (time > 9.0 && !printed.getValue())
         {
            printed.setTrue();
            LogTools.info(String.format("Should slide: %s box CoF: %s slope CoF: %s distance: %s",
                                        shouldSlide,
                                        boxFriction,
                                        groundFriction,
                                        expectedPosition.distance(floatingRootJoint.getJointPose().getPosition())));
         }
      });

      try
      {
         int numberOfSimulationTicks = 1000;
         LogTools.info(String.format("Doing slide test for %s s. Should slide: %s, box CoF: %s, slope CoF: %s, ground angle: %s, threshold: %s",
                                     numberOfSimulationTicks * dt,
                                     shouldSlide,
                                     boxFriction,
                                     groundFriction,
                                     groundPitch,
                                     slopeThreshold));
         simulationSession.getSimulationSessionControls().simulateNow(numberOfSimulationTicks);
         if (caughtException.getValue() != null)
            throw caughtException.getValue();
      }
      finally
      {
         if (VISUALIZE)
         {
            visualizerControls.waitUntilVisualizerDown();
         }
      }

      simulationSession.shutdownSession();
   }

   @Test
   public void testCoefficientOfFrictionRandom()
   {
      Random random = new Random(1254147);
      
      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);
      double groundHeight = 0.01;
      String name = "box";

      for (int i = 0; i <= ITERATIONS; i++)
      {
         double angleOfGround = random.nextDouble() * MAX_GROUND_ANGLE_FOR_RANDOM;
         double friction = random.nextDouble();
         double groundPitch = Math.toRadians(angleOfGround);

         RobotDefinition boxRobot = newBoxRobot(name, boxSize, 150.0, 0.8, null);
         SixDoFJointState initialState = new SixDoFJointState();
         Point3D initialPosition = new Point3D(0.0, 0.0, calculateZ(groundPitch));
         initialState.setConfiguration(new YawPitchRoll(0, groundPitch, 0), initialPosition);
         initialState.setVelocity(new Vector3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
         boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialState);

         boxRobot.getRigidBodyDefinition("boxRigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize)));

         GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, groundHeight);
         RigidBodyTransform terrainPose = new RigidBodyTransform();
         terrainPose.appendPitchRotation(groundPitch);
         terrainPose.appendTranslation(0.0, 0.0, 0.0);
         TerrainObjectDefinition terrain = new TerrainObjectDefinition();
         terrain.addCollisionShapeDefinition(new CollisionShapeDefinition(terrainPose, terrainGeometry));

         BulletMultiBodyJointParameters bulletMultiBodyJointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
         bulletMultiBodyJointParameters.setJointFriction(friction);

         SimulationSession simulationSession
               = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(BulletMultiBodyParameters.defaultBulletMultiBodyParameters(),
                                                                                                bulletMultiBodyJointParameters));
         int numberOfSimulationTicks = 1000;
         simulationSession.addRobot(boxRobot);
         simulationSession.addTerrainObject(terrain);

         if (simulationSession.getPhysicsEngine() instanceof BulletPhysicsEngine)
         {
            BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
            bulletPhysicsEngine.getTerrainObjects().get(0).getBtRigidBody().setFriction(1.0);
         }

         YoPoint3D expectedPosition = new YoPoint3D("expectedSpherePosition", simulationSession.getRootRegistry());
         SimFloatingRootJoint floatingRootJoint = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(0).getJoint(name);
         expectedPosition.set(initialPosition);

         for (int j = 0; j <= numberOfSimulationTicks; j++)
            simulationSession.runTick();

         if (friction > Math.abs(Math.tan(groundPitch)))
            EuclidCoreTestTools.assertEquals(expectedPosition, floatingRootJoint.getJointPose().getPosition(), EPSILON);
         else
         {
            Assertions.assertNotEquals(expectedPosition.getZ(), floatingRootJoint.getJointPose().getPosition().getZ());
         }

         simulationSession.shutdownSession();
      }
   }
   
   public static double calculateZ(double angleOfGround)
   {
      double a1 = 50.0 * Math.sin(angleOfGround);
      double b1 = 50.0 * Math.cos(angleOfGround);
      double a2 = b1 * Math.tan(angleOfGround - 0.00412);
      return a1 - a2;
   }

   public static RobotDefinition newBoxRobot(String name, Vector3D boxSize, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);

      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * boxSize.getX(),
                                                                                     radiusOfGyrationPercent * boxSize.getY(),
                                                                                     radiusOfGyrationPercent * boxSize.getZ()));

      if (color != null)
      {
         VisualDefinitionFactory factory = new VisualDefinitionFactory();
         factory.addBox(boxSize.getX(), boxSize.getY(), boxSize.getZ(), new MaterialDefinition(color));
         rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      }

      rootJoint.setSuccessor(rigidBody);
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }
}
