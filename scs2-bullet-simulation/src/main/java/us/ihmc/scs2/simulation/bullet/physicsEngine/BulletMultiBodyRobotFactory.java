package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;
import java.util.List;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundFromGimpactShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btConvexTriangleMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btGImpactMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;

public class BulletMultiBodyRobotFactory
{
   private static RigidBodyDefinition rigidBodyDefinition;
   private static HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private static int linkCountingIndex = 0;

   public static BulletMultiBodyRobot newInstance(Robot robot, YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      int numberOfLinks = countJoints(robot.getRootBody().getChildrenJoints().get(0)) - 1;
      System.out.println(numberOfLinks);
      float rootBodyMass = (float) robot.getRobotDefinition().getRootBodyDefinition().getMass();
      Vector3 rootBodyIntertia = new Vector3((float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM00(),
                                             (float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM11(),
                                             (float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM22());

      BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks, rootBodyMass, rootBodyIntertia, bulletMultiBodyParameters);
      
      JointBasics rootJoint = robot.getRootBody().getChildrenJoints().get(0);
      if (!(rootJoint instanceof SimFloatingRootJoint))
         throw new RuntimeException("Expecting a SimFloatingRootJoint, not a " + rootJoint.getClass().getSimpleName());
      
      rigidBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0).getSuccessor();
      
      //Create Bullet BaseCollider
      if (robot.getRootBody() != null)
      {
         createBulletCollider(bulletMultiBodyRobot, 
                              rigidBodyDefinition.getCollisionShapeDefinitions(),
                              rootJoint.getFrameAfterJoint(),
                              robot.getRootBody().getBodyFixedFrame(),
                              -1, 
                              bulletMultiBodyParameters.getJointFriction());
      }
      
      createLinkColliders(bulletMultiBodyRobot, robot.getRootBody().getChildrenJoints().get(0), rigidBodyDefinition, bulletMultiBodyParameters.getJointFriction());

      return bulletMultiBodyRobot;
   }
   
   private static void createLinkColliders(BulletMultiBodyRobot bulletMultiBodyRobot, JointBasics joint, RigidBodyDefinition jointRigidBodyDefinition, float friction)
   {
       for (JointBasics childJoint : joint.getSuccessor().getChildrenJoints())
       {

          System.out.println("child joint: " + childJoint.getName());
          
          for (JointDefinition childJointDefinition : jointRigidBodyDefinition.getChildrenJoints())
          {
             System.out.println("child joint definition: " + childJointDefinition.getName());
             if (childJoint.getName().equals(childJointDefinition.getName()))
             {
                createBulletCollider(bulletMultiBodyRobot, 
                                     childJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                     childJoint.getFrameAfterJoint(),
                                     childJoint.getSuccessor().getBodyFixedFrame(),
                                     jointNameToBulletJointIndexMap.get(childJoint.getName()), 
                                     friction);
                
                createLinkColliders(bulletMultiBodyRobot, childJoint, childJointDefinition.getSuccessor(), friction);
             }
          }
       }
   }

   private static btMultiBodyLinkCollider createBulletCollider(BulletMultiBodyRobot bulletMultiBodyRobot, 
                                                               List<CollisionShapeDefinition> collisionShapeDefinitions, 
                                                               ReferenceFrame frameAfterParentJoint,
                                                               ReferenceFrame linkCenterOfMassFrame, 
                                                               int bulletJointIndex, 
                                                               float friction)
   {
      btMultiBodyLinkCollider bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultiBodyRobot.getBulletMultiBodyRobot(), bulletJointIndex);
      
      btCompoundShape bulletCompoundShape = new btCompoundShape();
      
      for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
      {
         System.out.println("shapeDefinition Name: " + shapeDefinition.getName());
         btCollisionShape bulletCollisionShape = createBulletCollisionShape(shapeDefinition);
         bulletCompoundShape.addChildShape(getBulletCollisionShapeLocalTransform(shapeDefinition.getOriginPose(), frameAfterParentJoint, linkCenterOfMassFrame), bulletCollisionShape);
      }
      
      bulletMultiBodyLinkCollider.setCollisionShape(bulletCompoundShape);
      bulletMultiBodyLinkCollider.setFriction(friction);

      // TODO: come up with a way to store the collision Group/Mask to be used to add to the bullet physics manager in that class
      //		bulletPhysicsManager.addMultiBodyCollisionShape(bulletMultiBodyLinkCollider, collisionGroup, collisionGroupMask);

      return bulletMultiBodyLinkCollider;
   }
   
   private static ReferenceFrame collisionShapeDefinitionFrame;
   private static RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();
   
   private static Matrix4 getBulletCollisionShapeLocalTransform (YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint,
                                             ReferenceFrame frameAfterParentJoint,
                                             ReferenceFrame linkCenterOfMassFrame)
   {
      Matrix4 bulletCollisionShapeLocalTransform = new Matrix4();

      collisionShapeDefinitionFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameAfterParentJoint,
                                                                                                               new RigidBodyTransform(collisionShapeToFrameAfterParentJoint.getRotation(),
                                                                                                                                      collisionShapeToFrameAfterParentJoint.getTranslation()));
      
      collisionShapeDefinitionFrame.getTransformToDesiredFrame(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, linkCenterOfMassFrame);
      BulletTools.toBullet(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, bulletCollisionShapeLocalTransform);

      return bulletCollisionShapeLocalTransform;
      
   }
   
   private static btCollisionShape createBulletCollisionShape (CollisionShapeDefinition collisionShapeDefinition)
   {
      btCollisionShape bulletCollisionShape = null;
       
      // Just need to make sure the vertices for the libGDX shapes and the bullet shapes are the same
      //Color color = new Color(Color.WHITE);
      // TODO: Get to this later for the fingers
      if (collisionShapeDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShapeDefinition.getGeometryDefinition();
//         List<btConvexHullShape> shapes = BulletTools.loadConvexHullShapeFromFile(modelFileGeometryDefinition.getFileName());
//         List<btConvexPointCloudShape> shapes = BulletTools.loadConvexPointCloudShapesFromFile(modelFileGeometryDefinition.getFileName());

         Matrix4 identity = new Matrix4();
         if (collisionShapeDefinition.isConcave())
         {
            List<btGImpactMeshShape> shapes = BulletTools.loadConcaveGImpactMeshShapeFromFile(modelFileGeometryDefinition.getFileName());
            btCompoundFromGimpactShape compoundFromGimpactShape = new btCompoundFromGimpactShape();

            for (btCollisionShape shape : shapes)
            {
               shape.setMargin(0.01f);
               compoundFromGimpactShape.addChildShape(identity, shape);
            }

            bulletCollisionShape = compoundFromGimpactShape;
         }
         else
         {
            List<btConvexTriangleMeshShape> shapes = BulletTools.loadConvexTriangleMeshShapeFromFile(modelFileGeometryDefinition.getFileName());

            btCompoundShape compoundShape = new btCompoundShape();

            for (btCollisionShape shape : shapes)
            {
               shape.setMargin(0.01f);
               compoundShape.addChildShape(identity, shape);
            }

            bulletCollisionShape = compoundShape;
         }
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Box3DDefinition)
      {
         Box3DDefinition boxGeometryDefinition = (Box3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btBoxShape boxShape = new btBoxShape(new Vector3((float) boxGeometryDefinition.getSizeX() / 2.0f,
                                                          (float) boxGeometryDefinition.getSizeY() / 2.0f,
                                                          (float) boxGeometryDefinition.getSizeZ() / 2.0f));
         bulletCollisionShape = boxShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Sphere3DDefinition)
      {
         Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btSphereShape sphereShape = new btSphereShape((float) sphereGeometryDefinition.getRadius());
         bulletCollisionShape = sphereShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cylinder3DDefinition)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = (Cylinder3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btCylinderShapeZ cylinderShape = new btCylinderShapeZ(new Vector3((float) cylinderGeometryDefinition.getRadius(),
                                                                           (float) cylinderGeometryDefinition.getRadius(),
                                                                           (float) cylinderGeometryDefinition.getLength() / 2.0f));
         bulletCollisionShape = cylinderShape;
      }
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsuleGeometryDefinition = (Capsule3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         if (capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusY()
               || capsuleGeometryDefinition.getRadiusX() != capsuleGeometryDefinition.getRadiusZ()
               || capsuleGeometryDefinition.getRadiusY() != capsuleGeometryDefinition.getRadiusZ())
            LogTools.warn("Bullet capsule does not fully represent the intended capsule!");
         btCapsuleShapeZ capsuleShape = new btCapsuleShapeZ((float) capsuleGeometryDefinition.getRadiusX(), (float) capsuleGeometryDefinition.getLength());
         bulletCollisionShape = capsuleShape;
      }
      else
      {
         LogTools.warn("Implement collision for {}", collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName());
      }
      
      return bulletCollisionShape;  
   }

   private static int countJoints(JointBasics joint)
   {
      jointNameToBulletJointIndexMap.put(joint.getName(), linkCountingIndex-1);
      ++linkCountingIndex;
      
      int numberOfJoints = 1;
      for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJoints(childrenJoint);
      }
      return numberOfJoints;
   }

}
