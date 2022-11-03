package us.ihmc.scs2.examples.simulations.bullet;

import static us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools.newSphereRobot;

import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyDynamicsWorld;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;

public class FallingSphereExperimentalBulletSimulation
{
   public FallingSphereExperimentalBulletSimulation()
   {
      SimulationSession simulationSession = createSession();

      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
      btMultiBodyDynamicsWorld multiBodyDynamicsWorld = bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld().getBtMultiBodyDynamicsWorld();

      int size = multiBodyDynamicsWorld.getCollisionObjectArray().size();
      for (int i = 0; i < size; i++)
      {
         btCollisionShape collisionShape = multiBodyDynamicsWorld.getCollisionObjectArray().get(i).getCollisionShape();
         System.out.println(i + " before Visualizer " + collisionShape.getShapeType());
      }

      SessionVisualizer.startSessionVisualizer(simulationSession);

      size = multiBodyDynamicsWorld.getCollisionObjectArray().size();
      for (int i = 0; i < size; i++)
      {
         btCollisionShape collisionShape = multiBodyDynamicsWorld.getCollisionObjectArray().get(i).getCollisionShape();
         System.out.println(i + " after Visualizer " + collisionShape.getShapeType());
      }
   }

   public static SimulationSession createSession()
   {
      double radius1 = 0.3;
      double mass1 = 2.0;
      double radiusOfGyrationPercent1 = 1.0;
      ColorDefinition appearance1 = ColorDefinitions.DarkGreen();
      ColorDefinition stripesAppearance1 = ColorDefinitions.LightGreen();
      RobotDefinition sphereRobot1 = newSphereRobot("sphere1", radius1, mass1, radiusOfGyrationPercent1, appearance1, true, stripesAppearance1);

      sphereRobot1.getRigidBodyDefinition("sphere1RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(radius1)));

      SixDoFJointState sphere1InitialState = new SixDoFJointState();
      sphere1InitialState.setConfiguration(null, new Point3D(0, 0, 2.0));
      sphere1InitialState.setVelocity(null, new Vector3D(0.0, 0.0, 0));
      sphereRobot1.getRootJointDefinitions().get(0).setInitialJointState(sphere1InitialState);

      BulletMultiBodyParameters bulletMultiBodyParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      bulletMultiBodyParameters.setLinearDamping(0);
      BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      bulletMultiBodyJointParameter.setJointRestitution(1.0);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(bulletMultiBodyParameters,
                                                                                                                           bulletMultiBodyJointParameter));
      simulationSession.addRobot(sphereRobot1);

      GeometryDefinition terrainGeometry = new Box3DDefinition(6, 6, 0.01);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, 0.0));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.LightSlateGray())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);

      simulationSession.submitBufferSizeRequest(245760);
      simulationSession.setBufferRecordTickPeriod(8);
      //simulationSession.setSessionDTSeconds(0.000001);
      return simulationSession;
   }

   public static void main(String[] args)
   {
      new FallingSphereExperimentalBulletSimulation();
   }

}
