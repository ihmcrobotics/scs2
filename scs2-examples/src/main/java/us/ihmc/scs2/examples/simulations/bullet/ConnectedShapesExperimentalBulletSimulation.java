package us.ihmc.scs2.examples.simulations.bullet;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.DynamicsJNI;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyPoint2Point;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.math.Quaternion;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
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
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletBasedPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletSimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTools;

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
      
      int numberOfLinks = 1;
      boolean fixedBase = false;
      boolean canSleep = false;

      BulletSimulationSession simulationSession = new BulletSimulationSession((frame, rootRegistry) -> new BulletBasedPhysicsEngine(frame, rootRegistry));
      
      Vector3D connectionOffset = new Vector3D(0.9, 0.0, 0.0);

      RobotDefinition robotDefinition = new RobotDefinition("ConnectedShapes");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      RigidBodyDefinition rigidBody1 = ExampleExperimentalSimulationTools.newBoxRigidBody("box1", boxSize1, boxMass1, radiusOfGyrationPercent, boxApp1);
      rootJointDefinition.setSuccessor(rigidBody1);
      
      Vector3 baseInertiaDiagonal = new Vector3(0.0f, 0.0f, 0.0f);
      
      btCollisionShape boxCollisionShape = new btBoxShape(new Vector3((float)boxSize1.getX() / 2.0f, (float)boxSize1.getY() / 2.0f, (float)boxSize1.getZ()/2.0f));
      boxCollisionShape.calculateLocalInertia((float)boxMass1, baseInertiaDiagonal);
      btMultiBody bulletMultiBodyRobot = new btMultiBody(numberOfLinks, (float)boxMass1, baseInertiaDiagonal, fixedBase, canSleep);
      bulletMultiBodyRobot.setBasePos(new Vector3());
      bulletMultiBodyRobot.setWorldToBaseRot(new Quaternion());
      bulletMultiBodyRobot.setHasSelfCollision(true);
      bulletMultiBodyRobot.setUseGyroTerm(true);

      btMultiBodyLinkCollider baseCollider = new btMultiBodyLinkCollider(bulletMultiBodyRobot, -1);
      baseCollider.setCollisionShape(boxCollisionShape);
      baseCollider.setFriction(1.0f);
      bulletMultiBodyRobot.setBaseCollider(baseCollider);

      //bulletMultiBodyRobot.getBaseCollider().setCollisionShape(boxCollisionShape);
      
      RevoluteJointDefinition pinJoint = new RevoluteJointDefinition("pin");
      pinJoint.setAxis(Axis3D.Y);
      RigidBodyDefinition rigidBody2 = ExampleExperimentalSimulationTools.newBoxRigidBody("box2", boxSize2, boxMass2, radiusOfGyrationPercent, boxApp2);
      rigidBody2.setCenterOfMassOffset(connectionOffset);
      VisualDefinitionFactory factory2 = new VisualDefinitionFactory();
      factory2.appendTranslation(0.5 * connectionOffset.getX(), 0, 0);
      factory2.appendRotation(0.5 * Math.PI, Axis3D.Y);
      factory2.addCylinder(connectionOffset.getX(), 0.02, new MaterialDefinition(ColorDefinitions.Chocolate()));
      rigidBody2.getVisualDefinitions().forEach(visual -> visual.getOriginPose().prependTranslation(connectionOffset));
      rigidBody2.addVisualDefinitions(factory2.getVisualDefinitions());
      pinJoint.setSuccessor(rigidBody2);
      rigidBody1.addChildJoint(pinJoint);
      
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      SixDoFJointState initialRootJointState = new SixDoFJointState(null, new Point3D(0.0, 0.0, boxSize1.getZ()));
      rootJointDefinition.setInitialJointState(initialRootJointState);
      OneDoFJointState initialPinJointState = new OneDoFJointState();
      initialPinJointState.setEffort(3.0);
      pinJoint.setInitialJointState(initialPinJointState);
      
      int linkIndex = 0;
      int parentIndex = -1;
      Quaternion rotationFromParent = new Quaternion();
      Vector3 offsetOfPivotFromParentCenterOfMass = new Vector3((float)rigidBody1.getCenterOfMassOffset().getX(), (float)rigidBody1.getCenterOfMassOffset().getY(), (float)rigidBody1.getCenterOfMassOffset().getZ()); //Vector3(0.0f, 0.0f, -linkHalfExtents.z);
      Vector3 offsetOfCenterOfMassFromPivot = new Vector3((float)rigidBody2.getCenterOfMassOffset().getX(), (float)rigidBody2.getCenterOfMassOffset().getY(), (float)rigidBody2.getCenterOfMassOffset().getZ());
      boolean disableParentCollision = true;
      bulletMultiBodyRobot.setupSpherical(linkIndex,
                               (float)boxMass2,
                               baseInertiaDiagonal,
                               parentIndex,
                               rotationFromParent,
                               offsetOfPivotFromParentCenterOfMass,
                               offsetOfCenterOfMassFromPivot,
                               disableParentCollision);

      btMultiBodyLinkCollider linkCollider = new btMultiBodyLinkCollider(bulletMultiBodyRobot, linkIndex);
      btCollisionShape linkBox = new btBoxShape(new Vector3((float)boxSize2.getX() / 2.0f, (float)boxSize2.getY() / 2.0f, (float)boxSize2.getZ()/2.0f));
      linkCollider.setCollisionShape(linkBox);
      linkCollider.setFriction(1.0f);
      //linkCollider.setWorldTransform(worldTransform);
      bulletMultiBodyRobot.getLink(linkIndex).setCollider(linkCollider);
      simulationSession.addBulletMultiBodyRobot(robotDefinition, bulletMultiBodyRobot);

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      
   
      btCollisionShape terrainShape = new btBoxShape(new Vector3((float)groundLength/2.0f, (float)groundWidth/2.0f, 0.1f / 2.0f));
      Matrix4 terrainTransform = new Matrix4(); 
      BulletTools.toBullet(terrainPose, terrainTransform); 
      simulationSession.addTerrainObject(terrain, terrainShape, terrainTransform);
      
      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
   
   public static void main(String[] args)
   {
      new ConnectedShapesExperimentalBulletSimulation();
   }
}
