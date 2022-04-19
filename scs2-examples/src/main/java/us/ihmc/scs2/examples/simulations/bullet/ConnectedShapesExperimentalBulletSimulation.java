package us.ihmc.scs2.examples.simulations.bullet;

import javafx.application.Platform;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletDebugDrawingNode;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ConnectedShapesExperimentalBulletSimulation
{
   public ConnectedShapesExperimentalBulletSimulation()
   {  
      Vector3D boxSize1 = new Vector3D(0.5, 0.3, 0.3);
      double boxMass1 = 1.0;
      double radiusOfGyrationPercent = 0.8;
      ColorDefinition boxApp1 = ColorDefinitions.LightSeaGreen();

      Vector3D boxSize2 = new Vector3D(0.3, 0.3, 0.3);
      double boxMass2 = 1.0;
      ColorDefinition boxApp2 = ColorDefinitions.Teal();
      
      double groundWidth = 5.0;
      double groundLength = 5.0;

   
      Vector3D connectionOffset = new Vector3D(0.9, 0.0, 0.0);

      RobotDefinition robotDefinition = new RobotDefinition("ConnectedShapes");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      RigidBodyDefinition rigidBody1 = ExampleExperimentalSimulationTools.newBoxRigidBody("box1", boxSize1, boxMass1, radiusOfGyrationPercent, boxApp1);
      rigidBody1.getInertiaPose().getTranslation().add(0.01, -0.02, 0.03);
      rootJointDefinition.setSuccessor(rigidBody1);
      
      RevoluteJointDefinition pinJointDefinition = new RevoluteJointDefinition("pin");
      pinJointDefinition.setAxis(Axis3D.Y);
      RigidBodyDefinition rigidBody2 = ExampleExperimentalSimulationTools.newBoxRigidBody("box2", boxSize2, boxMass2, radiusOfGyrationPercent, boxApp2);
      rigidBody2.setCenterOfMassOffset(connectionOffset);
      VisualDefinitionFactory factory2 = new VisualDefinitionFactory();
      factory2.appendTranslation(0.5 * connectionOffset.getX(), 0, 0);
      factory2.appendRotation(0.5 * Math.PI, Axis3D.Y);
      factory2.addCylinder(connectionOffset.getX(), 0.02, new MaterialDefinition(ColorDefinitions.Chocolate()));
      rigidBody2.getVisualDefinitions().forEach(visual -> visual.getOriginPose().prependTranslation(connectionOffset));
      rigidBody2.addVisualDefinitions(factory2.getVisualDefinitions());
      //rigidBody2.getInertiaPose().getTranslation().add(-0.02, 0.01, -0.03);
      pinJointDefinition.setSuccessor(rigidBody2);
      rigidBody1.addChildJoint(pinJointDefinition);
      
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      SixDoFJointState initialRootJointState = new SixDoFJointState(null, new Point3D(0.0, 0.0, boxSize1.getZ()));
      rootJointDefinition.setInitialJointState(initialRootJointState);
      OneDoFJointState initialPinJointState = new OneDoFJointState();
      initialPinJointState.setEffort(3.0);
      pinJointDefinition.setInitialJointState(initialPinJointState);

      rigidBody1.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize1)));
      rigidBody2.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), connectionOffset),
                                                                          new Box3DDefinition(boxSize2)));
      
      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory());

      simulationSession.addRobot(robotDefinition);

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);

      SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
      
      sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.3, 0.0, 1.0);
      sessionVisualizer.getSessionVisualizerControls().setCameraPosition(7.0, 4.0, 3.0);
      sessionVisualizer.getToolkit().getSession().runTick();
   }
   
   public static void main(String[] args)
   {
      new ConnectedShapesExperimentalBulletSimulation();
   }
}
