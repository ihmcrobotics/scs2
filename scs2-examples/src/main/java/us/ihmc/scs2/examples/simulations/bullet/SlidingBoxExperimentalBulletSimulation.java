package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;

public class SlidingBoxExperimentalBulletSimulation
{
   public SlidingBoxExperimentalBulletSimulation()
   {
      SessionVisualizer.startSessionVisualizer(createSession());
   }

   public static SimulationSession createSession()
   {
      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);

      double groundPitch = Math.toRadians(34);
      double groundHeight = 0.01;

      RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box", boxSize, 150.0, 0.8, ColorDefinitions.DarkCyan());
      SixDoFJointState initialState = new SixDoFJointState();
      Point3D initialPosition = new Point3D(0, 0, 0.248); 
      initialState.setConfiguration(new YawPitchRoll(0, groundPitch, 0), initialPosition);
      initialState.setVelocity(new Vector3D(0, 0, 0), new Vector3D(0, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialState);

      boxRobot.getRigidBodyDefinition("boxRigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize)));

      GeometryDefinition terrainGeometry = new Box3DDefinition(100.0, 100.0, groundHeight);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.appendPitchRotation(groundPitch);
      terrainPose.appendTranslation(0, 0, 0);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.Lavender())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      BulletMultiBodyJointParameters bulletMultiBodyJointParameter = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      bulletMultiBodyJointParameter.setJointFriction(0.6);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(BulletMultiBodyParameters.defaultBulletMultiBodyParameters(),
                                                                                                                           bulletMultiBodyJointParameter));
      simulationSession.addRobot(boxRobot);
      simulationSession.addTerrainObject(terrain);
      return simulationSession;
   }

   public static void main(String[] args)
   {
      new SlidingBoxExperimentalBulletSimulation();
   }
}
