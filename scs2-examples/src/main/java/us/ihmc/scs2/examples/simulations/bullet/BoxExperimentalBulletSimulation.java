package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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

public class BoxExperimentalBulletSimulation
{
   public BoxExperimentalBulletSimulation()
   {
      SessionVisualizer.startSessionVisualizer(createSession());
   }

   public static SimulationSession createSession()
   {
      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory());

      RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box",
                                                                                boxXLength,
                                                                                boxYWidth,
                                                                                boxZHeight,
                                                                                boxMass,
                                                                                boxRadiusOfGyrationPercent,
                                                                                ColorDefinitions.DarkCyan());

      RigidBodyTransform boxRobotTransform = new RigidBodyTransform(new YawPitchRoll(0, 0, initialBoxRoll),
                                                                    new Point3D(0.0,
                                                                                groundWidth / 2.0 - 0.002,
                                                                                boxZHeight / 2.0 * 1.05
                                                                                      + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll))));
      boxRobot.getRigidBodyDefinition("boxRigidBody")
              .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxXLength, boxYWidth, boxZHeight)));
      SixDoFJointState initialJointState = new SixDoFJointState(boxRobotTransform.getRotation(), boxRobotTransform.getTranslation());

      initialJointState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      simulationSession.addRobot(boxRobot);

      RobotDefinition boxRobot2 = ExampleExperimentalSimulationTools.newBoxRobot("box2",
                                                                                 boxXLength,
                                                                                 boxYWidth,
                                                                                 boxZHeight,
                                                                                 boxMass,
                                                                                 0.01,
                                                                                 ColorDefinitions.Red());

      RigidBodyTransform boxRobotTransform2 = new RigidBodyTransform(new YawPitchRoll(0, 0, initialBoxRoll), new Point3D(0.0, 0.0, 0.5));

      boxRobot2.getRigidBodyDefinition("box2RigidBody")
               .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxXLength, boxYWidth, boxZHeight)));

      SixDoFJointState initialJointState2 = new SixDoFJointState(boxRobotTransform2.getRotation(), boxRobotTransform2.getTranslation());
      initialJointState2.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot2.getRootJointDefinitions().get(0).setInitialJointState(initialJointState2);

      simulationSession.addRobot(boxRobot2);

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -0.05));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);
      return simulationSession;
   }

   public static void main(String[] args)
   {
      new BoxExperimentalBulletSimulation();
   }
}