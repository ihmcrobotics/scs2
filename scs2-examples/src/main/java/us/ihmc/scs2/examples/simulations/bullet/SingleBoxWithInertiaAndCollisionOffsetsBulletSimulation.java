package us.ihmc.scs2.examples.simulations.bullet;

import javafx.application.Platform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletDebugDrawingNode;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

public class SingleBoxWithInertiaAndCollisionOffsetsBulletSimulation
{
   public SingleBoxWithInertiaAndCollisionOffsetsBulletSimulation()
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

      double intertiaPoseX = 0.05;
      double intertiaPoseY = 0.08;
      double intertiaPoseZ = 0.09;
      double intertiaPoseYaw = 0.01;
      double intertiaPosePitch = 0.01;
      double intertiaPoseRoll = 0.01;
      double collisionShapePoseX = intertiaPoseX - 0.01;
      double collisionShapePoseY = intertiaPoseY - 0.01;
      double collisionShapePoseZ = intertiaPoseZ - 0.01;
      double collisionShapePoseYaw = intertiaPoseYaw - 0.004;
      double collisionShapePosePitch = intertiaPosePitch - 0.004;
      double collisionShapePoseRoll = intertiaPoseRoll - 0.004;
      // Expressed in frame after joint
      YawPitchRollTransformDefinition inertiaPose = new YawPitchRollTransformDefinition(intertiaPoseX,
                                                                                        intertiaPoseY,
                                                                                        intertiaPoseZ,
                                                                                        intertiaPoseYaw,
                                                                                        intertiaPosePitch,
                                                                                        intertiaPoseRoll);
      // Expressed in frame after joint
      YawPitchRollTransformDefinition collisionShapePose = new YawPitchRollTransformDefinition(collisionShapePoseX,
                                                                                                 collisionShapePoseY,
                                                                                                 collisionShapePoseZ,
                                                                                                 collisionShapePoseYaw,
                                                                                                 collisionShapePosePitch,
                                                                                                 collisionShapePoseRoll);


      SimulationSession simulationSession = new SimulationSession((frame, rootRegistry) -> new BulletPhysicsEngine(frame, rootRegistry, BulletMultiBodyParameters.defaultBulletMultiBodyParameters()));

      String name = "box";
      RobotDefinition boxRobot = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(boxMass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(boxMass,
                                                                                     boxRadiusOfGyrationPercent * boxXLength,
                                                                                     boxRadiusOfGyrationPercent * boxYWidth,
                                                                                     boxRadiusOfGyrationPercent * boxZHeight));
      rigidBody.getInertiaPose().set(collisionShapePose);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.appendTransform(collisionShapePose);
      factory.addBox(boxXLength, boxYWidth, boxZHeight, new MaterialDefinition(ColorDefinitions.DarkCyan()));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      rootJoint.setSuccessor(rigidBody);
      boxRobot.setRootBodyDefinition(rootBody);

      RigidBodyTransform boxRobotTransform = new RigidBodyTransform(new YawPitchRoll(0, 0, initialBoxRoll),
                                                                    new Point3D(0.0,
                                                                                0.3,
                                                                                0.3));
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(boxXLength, boxYWidth, boxZHeight));
      collisionShapeDefinition.getOriginPose().set(inertiaPose);
      boxRobot.getRigidBodyDefinition("boxRigidBody")
              .addCollisionShapeDefinition(collisionShapeDefinition);

      SixDoFJointState initialJointState = new SixDoFJointState(boxRobotTransform.getRotation(), boxRobotTransform.getTranslation());
      initialJointState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      simulationSession.addRobot(boxRobot);

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.05, 0.0, -0.05));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);

      simulationSession.initializeBufferSize(24000);
      BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
   }
   
   public static void main(String[] args)
   {
      new SingleBoxWithInertiaAndCollisionOffsetsBulletSimulation();
   }
}