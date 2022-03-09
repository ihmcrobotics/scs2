package us.ihmc.scs2.examples.simulations.bullet;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

public class StackOfShapesExperimentalBulletSimulation
{
   public StackOfShapesExperimentalBulletSimulation()
   {  
      double groundWidth = 5.0;
      double groundLength = 5.0;

      SimulationSession simulationSession = new SimulationSession((frame, rootRegistry) -> new BulletPhysicsEngine(frame, rootRegistry));

      int numberOfBlocks = 6;
      Random random = new Random(1886L);

      List<RobotDefinition> robotDefinitions = new ArrayList<>();

      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      double mass = 0.2;
//      String typeShape = "BOX";
//      String typeShape = "SPHERE";
//      String typeShape = "CYLINDER";
      String typeShape = "CAPSULE";

      for (int i = 0; i < numberOfBlocks; i++)
      {
         RobotDefinition shapeRobot = new RobotDefinition();
         
         ColorDefinition appearance = ColorDefinition.rgb(random.nextInt());
         if (typeShape == "BOX")
         {
            shapeRobot = ExampleExperimentalSimulationTools.newBoxRobot("Block" + i, boxSizeX, boxSizeY, boxSizeZ, mass, 0.5, appearance); 
            shapeRobot.getRigidBodyDefinition("Block" + i + "RigidBody")
            .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSizeX, boxSizeY, boxSizeZ)));
         }
         else if (typeShape == "SPHERE")
         {
            shapeRobot = ExampleExperimentalSimulationTools.newSphereRobot("Sphere" + i, boxSizeX, mass, 0.5, appearance, false, appearance);
            shapeRobot.getRigidBodyDefinition("Sphere" + i + "RigidBody")
            .addCollisionShapeDefinition(new CollisionShapeDefinition(new Sphere3DDefinition(boxSizeX)));
         }
         else if (typeShape == "CYLINDER")
         {
            shapeRobot = ExampleExperimentalSimulationTools.newCylinderRobot("Cylinder" + i, boxSizeY, boxSizeZ, mass, 0.5, appearance, false, appearance);
            shapeRobot.getRigidBodyDefinition("Cylinder" + i + "RigidBody")
            .addCollisionShapeDefinition(new CollisionShapeDefinition(new Cylinder3DDefinition(boxSizeZ, boxSizeY)));
         }
         else 
         {
            shapeRobot = ExampleExperimentalSimulationTools.newCapsuleRobot("Capsule" + i, boxSizeY, boxSizeZ/2.0, mass, 0.5, appearance, false, appearance);
            shapeRobot.getRigidBodyDefinition("Capsule" + i + "RigidBody")
            //.addCollisionShapeDefinition(new CollisionShapeDefinition(new Capsule3DDefinition(boxSizeZ/2.0, boxSizeY)));
            .addCollisionShapeDefinition(new CollisionShapeDefinition(new Capsule3DDefinition(boxSizeY, boxSizeZ/2.0)));
         }
         
         robotDefinitions.add(shapeRobot);
         
         double x = 0.0;
         double y = i * 0.01;
         double z = boxSizeZ * 2.1 * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
         double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

         shapeRobot.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z)));
      }
      
      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);
      robotDefinitions.forEach(simulationSession::addRobot);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
   
   public static void main(String[] args)
   {
      new StackOfShapesExperimentalBulletSimulation();
   }
}
