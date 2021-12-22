package us.ihmc.scs2.examples.simulations;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class StackOfBlocksExperimentalSimulation
{

   public StackOfBlocksExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.3);
      contactParameters.setRestitutionThreshold(0.15);
      contactParameters.setErrorReductionParameter(0.01);

      int numberOfBlocks = 6;
      Random random = new Random(1886L);

      List<RobotDefinition> robotDefinitions = new ArrayList<>();

      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      double mass = 0.2;

      for (int i = 0; i < numberOfBlocks; i++)
      {
         ColorDefinition appearance = ColorDefinition.rgb(random.nextInt());
         RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("Block" + i, boxSizeX, boxSizeY, boxSizeZ, mass, 0.5, appearance);
         robotDefinitions.add(boxRobot);

         boxRobot.getRigidBodyDefinition("Block" + i + "RigidBody")
                 .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSizeX, boxSizeY, boxSizeZ)));

         double x = 0.0;
         double y = 0.0;
         double z = boxSizeZ * 1.05 * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
         double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

         boxRobot.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z)));
      }

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      GeometryDefinition terrainGeometry = new Box3DDefinition(1000, 1000, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkGrey())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newImpulseBasedPhysicsEngineFactory(contactParameters));
      robotDefinitions.forEach(simulationSession::addRobot);
      simulationSession.addTerrainObject(terrain);
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new StackOfBlocksExperimentalSimulation();
   }
}
