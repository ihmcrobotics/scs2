package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;
import java.util.List;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundFromGimpactShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btConeShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btConvexTriangleMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btGImpactMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyJointLimitConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

public class BulletMultiBodyRobotFactory
{
   private static HashMap<String, Integer> jointNameToBulletJointIndexMap;
   private static int linkCountingIndex;
   private static int numberOfLinks;

   public static BulletMultiBodyRobot newInstance(Robot robot, YoBulletMultiBodyParameters bulletMultiBodyParameters, YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      JointBasics rootJoint = robot.getRootBody().getChildrenJoints().get(0);
      boolean hasBaseCollider = rootJoint instanceof SimFloatingRootJoint && robot.getRootBody().getChildrenJoints().size()==1;

      jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
      linkCountingIndex = hasBaseCollider ? 0 : 1;
      numberOfLinks = 0;
      for (JointBasics joint : robot.getRootBody().getChildrenJoints())
      {
         numberOfLinks += countJointsAndCreateIndexMap(joint) - (hasBaseCollider ? 1 : 0);
      }

      RigidBodyDefinition rigidBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0).getSuccessor();
      float rootBodyMass = (float) rigidBodyDefinition.getMass();
      Vector3 rootBodyIntertia = new Vector3((float) rigidBodyDefinition.getMomentOfInertia().getM00(),
                                             (float) rigidBodyDefinition.getMomentOfInertia().getM11(),
                                             (float) rigidBodyDefinition.getMomentOfInertia().getM22());

      BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks,
                                                                           rootBodyMass,
                                                                           rootBodyIntertia,
                                                                           bulletMultiBodyParameters.getFixedBase(),
                                                                           bulletMultiBodyParameters.getCanSleep(),
                                                                           jointNameToBulletJointIndexMap);

      

      //Create BaseCollider
      if (hasBaseCollider)
      {
         bulletMultiBodyRobot.getBulletMultiBody()
                             .setBaseCollider(createBulletCollider(bulletMultiBodyRobot,
                                                                   rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                                   rootJoint.getFrameAfterJoint(),
                                                                   rootJoint.getSuccessor().getBodyFixedFrame(),
                                                                   -1,
                                                                   rootJoint.getName()));
         //Create LinkColliders for each robot joint 
         createLinkColliders(bulletMultiBodyRobot, robot, rootJoint);
         
      } else
      {
         for (JointBasics joint : robot.getRootBody().getChildrenJoints())
         {
            int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getName());
            JointDefinition rootJointDefinition = robot.getRobotDefinition().getJointDefinition(joint.getName());
            btMultibodyLink bulletLink = setupLinkType(bulletMultiBodyRobot, joint, rootJointDefinition, linkIndex);
            bulletLink.setCollider(createBulletCollider(bulletMultiBodyRobot,
                                                        rootJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                        joint.getFrameAfterJoint(),
                                                        joint.getSuccessor().getBodyFixedFrame(),
                                                        linkIndex,
                                                        joint.getName()));
            
            //Create LinkColliders for each root joint 
            createLinkColliders(bulletMultiBodyRobot, robot, joint);
         }
      }

      bulletMultiBodyRobot.finalizeMultiDof(bulletMultiBodyParameters, bulletMultiBodyJointParameters);

      return bulletMultiBodyRobot;
   }

   private static void createLinkColliders(BulletMultiBodyRobot bulletMultiBodyRobot, Robot robot, JointBasics joint)
   {
      for (JointBasics childJoint : joint.getSuccessor().getChildrenJoints())
      {
         JointDefinition childJointDefinition = robot.getRobotDefinition().getJointDefinition(childJoint.getName());

         int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(childJoint.getName());

         btMultibodyLink bulletLink = setupLinkType(bulletMultiBodyRobot, childJoint, childJointDefinition, linkIndex);
         bulletLink.setCollider(createBulletCollider(bulletMultiBodyRobot,
                                                     childJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                     childJoint.getFrameAfterJoint(),
                                                     childJoint.getSuccessor().getBodyFixedFrame(),
                                                     linkIndex,
                                                     childJoint.getName()));

         createLinkColliders(bulletMultiBodyRobot, robot, childJoint);
      }
   }

   private static btMultibodyLink setupLinkType(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                JointBasics joint,
                                                JointDefinition jointDefinition,
                                                int bulletJointIndex)
   {
      Quaternion rotationFromParentGDX = new Quaternion();
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent = new us.ihmc.euclid.tuple4D.Quaternion(jointDefinition.getTransformToParent().getRotation());
      euclidRotationFromParent.invert();
      BulletTools.toBullet(euclidRotationFromParent, rotationFromParentGDX);

      RigidBodyTransform parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid = new RigidBodyTransform();
      joint.getPredecessor().getBodyFixedFrame().getTransformToDesiredFrame(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid,
                                                                            joint.getFrameBeforeJoint());
      parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.invert();
      Vector3 parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet = new Vector3();
      BulletTools.toBullet(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.getTranslation(),
                           parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet);

      RigidBodyTransform parentJointAfterFrameToLinkCenterOfMassTransformEuclid = new RigidBodyTransform();
      joint.getFrameAfterJoint().getTransformToDesiredFrame(parentJointAfterFrameToLinkCenterOfMassTransformEuclid, joint.getSuccessor().getBodyFixedFrame());
      parentJointAfterFrameToLinkCenterOfMassTransformEuclid.invert();
      Vector3 parentJointAfterFrameToLinkCenterOfMassTranslationBullet = new Vector3();
      BulletTools.toBullet(parentJointAfterFrameToLinkCenterOfMassTransformEuclid.getTranslation(), parentJointAfterFrameToLinkCenterOfMassTranslationBullet);

      float linkMass = (float) jointDefinition.getSuccessor().getMass();
      Vector3 baseInertiaDiagonal = new Vector3((float) jointDefinition.getSuccessor().getMomentOfInertia().getM00(),
                                                (float) jointDefinition.getSuccessor().getMomentOfInertia().getM11(),
                                                (float) jointDefinition.getSuccessor().getMomentOfInertia().getM22());

      if (joint instanceof SimRevoluteJoint)
      {
         RevoluteJointDefinition revoluteJointDefinition = (RevoluteJointDefinition) jointDefinition;

         int parentBulletJointIndex;
         if (joint.getPredecessor().getParentJoint() == null)
            parentBulletJointIndex = -1;
         else
            parentBulletJointIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getPredecessor().getParentJoint().getName());

         Vector3 jointAxis = new Vector3();
         BulletTools.toBullet(revoluteJointDefinition.getAxis(), jointAxis);
         boolean disableParentCollision = true;
         bulletMultiBodyRobot.getBulletMultiBody().setupRevolute(bulletJointIndex,
                                                                 linkMass,
                                                                 baseInertiaDiagonal,
                                                                 parentBulletJointIndex,
                                                                 rotationFromParentGDX,
                                                                 jointAxis,
                                                                 parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet,
                                                                 parentJointAfterFrameToLinkCenterOfMassTranslationBullet,
                                                                 disableParentCollision);
         
         btMultiBodyJointLimitConstraint multiBodyJointLimitConstraint = new btMultiBodyJointLimitConstraint(bulletMultiBodyRobot.getBulletMultiBody(),
                                                                                                             bulletJointIndex,
                                                                                                             (float) revoluteJointDefinition.getPositionLowerLimit(),
                                                                                                             (float) revoluteJointDefinition.getPositionUpperLimit());
         
         //multiBodyJointLimitConstraint.setMaxAppliedImpulse((float) revoluteJointDefinition.getEffortUpperLimit());

         bulletMultiBodyRobot.addMultiBodyConstraint(multiBodyJointLimitConstraint);
      }
      
      if (joint instanceof SimPrismaticJoint)
      {
         PrismaticJointDefinition PrimaticJointDefinition = (PrismaticJointDefinition) jointDefinition;

         int parentBulletJointIndex;
         if (joint.getPredecessor().getParentJoint() == null)
            parentBulletJointIndex = -1;
         else
            parentBulletJointIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getPredecessor().getParentJoint().getName());

         Vector3 jointAxis = new Vector3();
         BulletTools.toBullet(PrimaticJointDefinition.getAxis(), jointAxis);
         boolean disableParentCollision = true;
         bulletMultiBodyRobot.getBulletMultiBody().setupPrismatic(bulletJointIndex,
                                                                 linkMass,
                                                                 baseInertiaDiagonal,
                                                                 parentBulletJointIndex,
                                                                 rotationFromParentGDX,
                                                                 jointAxis,
                                                                 parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet,
                                                                 parentJointAfterFrameToLinkCenterOfMassTranslationBullet,
                                                                 disableParentCollision);
                  
         btMultiBodyJointLimitConstraint multiBodyJointLimitConstraint = new btMultiBodyJointLimitConstraint(bulletMultiBodyRobot.getBulletMultiBody(),
                                                                                                             bulletJointIndex,
                                                                                                             (float) PrimaticJointDefinition.getPositionLowerLimit(),
                                                                                                             (float) PrimaticJointDefinition.getPositionUpperLimit());

         
         bulletMultiBodyRobot.addMultiBodyConstraint(multiBodyJointLimitConstraint);
      }
      
      return bulletMultiBodyRobot.getBulletMultiBody().getLink(bulletJointIndex);
   }

   private static btMultiBodyLinkCollider createBulletCollider(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                               List<CollisionShapeDefinition> collisionShapeDefinitions,
                                                               ReferenceFrame frameAfterParentJoint,
                                                               ReferenceFrame linkCenterOfMassFrame,
                                                               int bulletJointIndex,
                                                               String jointName)
   {
      BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider = new BulletMultiBodyLinkCollider(bulletMultiBodyRobot.getBulletMultiBody(),
                                                                                                bulletJointIndex,
                                                                                                jointName);
      
      btCompoundShape bulletCompoundShape = new btCompoundShape();
      int collisionGroup = 2;
      int collisionGroupMask = 1 + 2;

      if (collisionShapeDefinitions.size() > 0)
      {
         collisionGroupMask = (int) collisionShapeDefinitions.get(0).getCollisionGroup();
         collisionGroup = (int) collisionShapeDefinitions.get(0).getCollisionMask();
      }

      bulletMultiBodyLinkCollider.setCollisionGroupMask(collisionGroup, collisionGroupMask);

      for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
      {
         btCollisionShape bulletCollisionShape = createBulletCollisionShape(shapeDefinition);
         bulletCompoundShape.addChildShape(bulletCollisionShapeLocalTransform(shapeDefinition.getOriginPose(), frameAfterParentJoint, linkCenterOfMassFrame),
                                           bulletCollisionShape);
      }

      bulletMultiBodyLinkCollider.setCollisionShape(bulletCompoundShape);

      bulletMultiBodyRobot.addBulletMuliBodyLinkCollider(bulletMultiBodyLinkCollider);

      return bulletMultiBodyLinkCollider.getMultiBodyLinkCollider();
   }

   private static Matrix4 bulletCollisionShapeLocalTransform = new Matrix4();
   private static RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();

   private static Matrix4 bulletCollisionShapeLocalTransform(YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint,
                                                             ReferenceFrame frameAfterParentJoint,
                                                             ReferenceFrame linkCenterOfMassFrame)
   {
      ReferenceFrame collisionShapeDefinitionFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(frameAfterParentJoint,
                                                                                                                              new RigidBodyTransform(collisionShapeToFrameAfterParentJoint.getRotation(),
                                                                                                                                                     collisionShapeToFrameAfterParentJoint.getTranslation()));

      collisionShapeDefinitionFrame.getTransformToDesiredFrame(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, linkCenterOfMassFrame);
      BulletTools.toBullet(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, bulletCollisionShapeLocalTransform);

      return bulletCollisionShapeLocalTransform;

   }

   private static btCollisionShape createBulletCollisionShape(CollisionShapeDefinition collisionShapeDefinition)
   {
      btCollisionShape bulletCollisionShape = null;

      if (collisionShapeDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShapeDefinition.getGeometryDefinition();

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
      else if (collisionShapeDefinition.getGeometryDefinition() instanceof Cone3DDefinition)
      {
         Cone3DDefinition coneGeometryDefinition = (Cone3DDefinition) collisionShapeDefinition.getGeometryDefinition();
         btConeShapeZ coneShape = new btConeShapeZ((float) coneGeometryDefinition.getRadius(), (float)coneGeometryDefinition.getHeight());
         bulletCollisionShape = coneShape;
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

   private static int countJointsAndCreateIndexMap(JointBasics joint)
   {
      jointNameToBulletJointIndexMap.put(joint.getName(), linkCountingIndex - 1);
      ++linkCountingIndex;

      int numberOfJoints = 1;
      for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJointsAndCreateIndexMap(childrenJoint);
      }
      return numberOfJoints;
   }

}
