package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletBasedPhysicsEngine;

public class BoxExperimentalSimulation
{
   
   public BoxExperimentalSimulation()
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

      SimulationSession simulationSession = new SimulationSession((frame, rootRegistry) -> new BulletBasedPhysicsEngine(frame, rootRegistry));
      
      RobotDefinition boxRobot = newBoxRobot("box", boxXLength, boxYWidth, boxZHeight, boxMass, boxRadiusOfGyrationPercent, ColorDefinitions.DarkCyan());
      
      RigidBodyTransform boxRobotTransform = new RigidBodyTransform(new YawPitchRoll(0, 0, initialBoxRoll), new Point3D(0.0,
                                                                                                                   groundWidth / 2.0 - 0.002,
                                                                                                                   boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll))));
      SixDoFJointState initialJointState = new SixDoFJointState(boxRobotTransform.getRotation(), boxRobotTransform.getTranslation());
      initialJointState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);
      
      simulationSession.addRobot(boxRobot);
   
      RigidBodyTransform boxRobotTransform2 = new RigidBodyTransform(new YawPitchRoll(0, 0, initialBoxRoll), new Point3D(0.0, 0.0, 0.5));
      RobotDefinition boxRobot2 = newBoxRobot("box2", boxXLength, boxYWidth, boxZHeight, boxMass, 0.0, ColorDefinitions.Red());
      SixDoFJointState initialJointState2 = new SixDoFJointState(boxRobotTransform2.getRotation(), boxRobotTransform2.getTranslation());
      boxRobot2.getRootJointDefinitions().get(0).setInitialJointState(initialJointState2);
     
      simulationSession.addRobot(boxRobot2);
      
      
      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -0.05));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);
      
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
   
   private static RobotDefinition newBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(newBoxRigidBody(name + "RigidBody", sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, color));
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }
   
   private static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                                     double radiusOfGyrationPercent, ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, null, color);
   }
   
   private static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                                     double radiusOfGyrationPercent, Vector3DReadOnly offsetFromParentJoint, ColorDefinition color)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(rigidBodyName);
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * sizeX,
                                                                                     radiusOfGyrationPercent * sizeY,
                                                                                     radiusOfGyrationPercent * sizeZ));
      if (offsetFromParentJoint != null)
         rigidBody.setCenterOfMassOffset(offsetFromParentJoint);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      if (offsetFromParentJoint != null)
         factory.appendTranslation(offsetFromParentJoint);
      factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(color));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }

   public static void main(String[] args)
   {
      new BoxExperimentalSimulation();
   }
}