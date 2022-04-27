package us.ihmc.scs2.examples.simulations.bullet;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;

public class StackOfBoxesExperimentalBulletSimulation
{
   private static final boolean DEBUG = false;
   
   public StackOfBoxesExperimentalBulletSimulation()
   {
      double groundWidth = 5.0;
      double groundLength = 5.0;

      int numberOfBlocks = 6;
      Random random = new Random(1886L);

      List<RobotDefinition> robotDefinitions = new ArrayList<>();

      double radiusOfGyrationPercent = 0.5;
      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      double mass = 0.2;

      double intertiaPoseX = 0.09;
      double intertiaPoseY = 0.03;
      double intertiaPoseZ = 0.01;
      double intertiaPoseYaw = 0.01;
      double intertiaPosePitch = 0.03;
      double intertiaPoseRoll = 0.05;

      YawPitchRollTransformDefinition inertiaPose = new YawPitchRollTransformDefinition(intertiaPoseX,
                                                                                        intertiaPoseY,
                                                                                        intertiaPoseZ,
                                                                                        intertiaPoseYaw,
                                                                                        intertiaPosePitch,
                                                                                        intertiaPoseRoll);

      for (int i = 0; i < numberOfBlocks; i++)
      {
         ColorDefinition appearance = ColorDefinition.rgb(random.nextInt());
         String name = "Block" + i;

         RobotDefinition boxRobot = new RobotDefinition(name);

         RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
         SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
         rootBody.addChildJoint(rootJoint);

         RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
         rigidBody.setMass(mass);
         rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                        radiusOfGyrationPercent * boxSizeX,
                                                                                        radiusOfGyrationPercent * boxSizeY,
                                                                                        radiusOfGyrationPercent * boxSizeZ));

         rigidBody.getInertiaPose().set(inertiaPose);

         VisualDefinitionFactory factory = new VisualDefinitionFactory();
         factory.appendTransform(inertiaPose);
         factory.addBox(boxSizeX, boxSizeY, boxSizeZ, new MaterialDefinition(appearance));
         rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
         rootJoint.setSuccessor(rigidBody);

         boxRobot.setRootBodyDefinition(rootBody);
         robotDefinitions.add(boxRobot);

         CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(boxSizeX, boxSizeY, boxSizeZ));
         collisionShapeDefinition.getOriginPose().set(inertiaPose);
         boxRobot.getRigidBodyDefinition(name + "RigidBody").addCollisionShapeDefinition(collisionShapeDefinition);

         double x = 0.00;
         double y = i * 0.02;
         double z = boxSizeZ * 2.1 * (i + 1.0);

         double yaw = 0.0;
         double pitch = 0.0; //RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
         double roll = 0.0; //RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

         boxRobot.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z)));

      }

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory());
      simulationSession.addTerrainObject(terrain);
      robotDefinitions.forEach(simulationSession::addRobot);

      if (!DEBUG)
         SessionVisualizer.startSessionVisualizer(simulationSession);
      else
      {
         SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
   
         sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.3, 0.0, 1.0);
         sessionVisualizer.getSessionVisualizerControls().setCameraPosition(7.0, 4.0, 3.0);
         sessionVisualizer.getToolkit().getSession().runTick();
      }
   }
   
   public static void main(String[] args)
   {
      new StackOfBoxesExperimentalBulletSimulation();
   }

}
