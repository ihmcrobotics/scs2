package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
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
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.euclid.YoPoint3D;

public class BulletCoefficientOfFrictionTest
{
   private static final double EPSILON = 0.01;
   private static final boolean VISUALIZE = false;
   private static final int ITERATIONS = 100;

   @Test
   public void testCoefficientOfFriction() throws Throwable
   {
      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);

      double angleOfGround = 34.0;
      Double friction = 0.7;
      double groundHeight = 0.01;
      double groundPitch = Math.toRadians(angleOfGround);
      String name = "box";
      double dt = 0.01;

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

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(BulletMultiBodyParameters.defaultBulletMultiBodyParameters(),
                                                                                                                           bulletMultiBodyJointParameters));
      int numberOfSimulationTicks = 1000;
      simulationSession.addRobot(boxRobot);
      simulationSession.addTerrainObject(terrain);
      simulationSession.setSessionDTSeconds(dt);

      SessionVisualizerControls visualizerControls = null;

      if (VISUALIZE)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         visualizerControls.waitUntilVisualizerFullyUp();
      }

      YoPoint3D expectedPosition = new YoPoint3D("expectedSpherePosition", simulationSession.getRootRegistry());
      SimFloatingRootJoint floatingRootJoint = (SimFloatingRootJoint) simulationSession.getPhysicsEngine().getRobots().get(0).getJoint(name);

      MutableObject<Throwable> caughtException = new MutableObject<>(null);
      simulationSession.addRunThrowableListener(t -> caughtException.setValue(t));
      simulationSession.addAfterPhysicsCallback(new TimeConsumer()
      {
         @Override
         public void accept(double time)
         {
            expectedPosition.set(initialPosition);
            if (friction > Math.abs(Math.tan(angleOfGround)))
               EuclidCoreTestTools.assertEquals(expectedPosition, floatingRootJoint.getJointPose().getPosition(), EPSILON);
            else
            {
               if (time > 0.5)
               {
                  Assertions.assertNotEquals(expectedPosition.getZ(), floatingRootJoint.getJointPose().getPosition().getZ());
               }

            }
         }
      });

      try
      {
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
         double angleOfGround = random.nextDouble() * 100 / 2;
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
