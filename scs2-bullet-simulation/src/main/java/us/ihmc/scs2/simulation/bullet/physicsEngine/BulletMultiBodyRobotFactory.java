package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.HashMap;
import java.util.List;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyJointLimitConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
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

   public static BulletMultiBodyRobot newInstance(Robot robot,
                                                  YoBulletMultiBodyParameters bulletMultiBodyParameters,
                                                  YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      if (robot.getRootBody().getChildrenJoints().size() == 0)
      {
         throw new UnsupportedOperationException("Robot must have at least one joint: " + robot.getClass().getSimpleName());
      }
      
      JointBasics rootJoint = robot.getRootBody().getChildrenJoints().get(0);
      boolean hasBaseCollider = rootJoint instanceof SimFloatingRootJoint;

      jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
      linkCountingIndex = hasBaseCollider ? 0 : 1;
      numberOfLinks = 0;
      for (JointBasics joint : robot.getRootBody().getChildrenJoints())
      {
         numberOfLinks += countJointsAndCreateIndexMap(joint) - (hasBaseCollider ? 1 : 0);
      }

      RigidBodyDefinition rigidBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0).getSuccessor();
      float rootBodyMass = (float) rigidBodyDefinition.getMass();
      
      Vector3 rootBodyIntertia = decomposeMomentOfInertia(rigidBodyDefinition);
      boolean fixedBase = hasBaseCollider ? false : true;
      BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks,
                                                                           rootBodyMass,
                                                                           rootBodyIntertia,
                                                                           fixedBase,
                                                                           bulletMultiBodyParameters.getCanSleep(),
                                                                           jointNameToBulletJointIndexMap);

      for (JointBasics joint : robot.getRootBody().getChildrenJoints())
      {
         int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getName());
         if (linkIndex == -1) 
         {
            //create BaseCollider
            bulletMultiBodyRobot.getBtMultiBody()
                                .setBaseCollider(createBulletLinkCollider(bulletMultiBodyRobot,
                                                                          rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                                          rootJoint.getSuccessor().getBodyFixedFrame(),
                                                                          -1,
                                                                          rootJoint.getName()));
         }
         else
         {
            JointDefinition rootJointDefinition = robot.getRobotDefinition().getJointDefinition(joint.getName());
            btMultibodyLink bulletLink = setupBtMultibodyLink(bulletMultiBodyRobot,
                                                              joint,
                                                              rootJointDefinition,
                                                              linkIndex,
                                                              bulletMultiBodyJointParameters.getJointDisableParentCollision());
            bulletLink.setCollider(createBulletLinkCollider(bulletMultiBodyRobot,
                                                            rootJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                            joint.getSuccessor().getBodyFixedFrame(),
                                                            linkIndex,
                                                            joint.getName()));
         }

         //Create LinkColliders for each root joint 
         createBulletLinkColliderChildren(bulletMultiBodyRobot, robot, joint, bulletMultiBodyJointParameters.getJointDisableParentCollision());
      }

      bulletMultiBodyRobot.finalizeMultiDof(bulletMultiBodyParameters, bulletMultiBodyJointParameters);

      return bulletMultiBodyRobot;
   }

   private static void createBulletLinkColliderChildren(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                        Robot robot,
                                                        JointBasics joint,
                                                        boolean disableParentCollision)
   {
      for (JointBasics childJoint : joint.getSuccessor().getChildrenJoints())
      {
         JointDefinition childJointDefinition = robot.getRobotDefinition().getJointDefinition(childJoint.getName());

         int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(childJoint.getName());

         btMultibodyLink btMultibodyLink = setupBtMultibodyLink(bulletMultiBodyRobot, childJoint, childJointDefinition, linkIndex, disableParentCollision);
         btMultibodyLink.setCollider(createBulletLinkCollider(bulletMultiBodyRobot,
                                                              childJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                              childJoint.getSuccessor().getBodyFixedFrame(),
                                                              linkIndex,
                                                              childJoint.getName()));

         createBulletLinkColliderChildren(bulletMultiBodyRobot, robot, childJoint, disableParentCollision);
      }
   }

   private static btMultibodyLink setupBtMultibodyLink(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                       JointBasics joint,
                                                       JointDefinition jointDefinition,
                                                       int bulletJointIndex,
                                                       boolean disableParentCollision)
   {
      Quaternion rotationFromParentBullet = new Quaternion();
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent = new us.ihmc.euclid.tuple4D.Quaternion(jointDefinition.getTransformToParent().getRotation());
      euclidRotationFromParent.invert();
      BulletTools.toBullet(euclidRotationFromParent, rotationFromParentBullet);

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
      
      Vector3 linkInertiaDiagonal = decomposeMomentOfInertia(jointDefinition.getSuccessor());

      float linkMass = (float) jointDefinition.getSuccessor().getMass();

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
 
         bulletMultiBodyRobot.getBtMultiBody().setupRevolute(bulletJointIndex,
                                                             linkMass,
                                                             linkInertiaDiagonal,
                                                             parentBulletJointIndex,
                                                             rotationFromParentBullet,
                                                             jointAxis,
                                                             parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet,
                                                             parentJointAfterFrameToLinkCenterOfMassTranslationBullet,
                                                             disableParentCollision);

         btMultiBodyJointLimitConstraint multiBodyJointLimitConstraint = new btMultiBodyJointLimitConstraint(bulletMultiBodyRobot.getBtMultiBody(),
                                                                                                             bulletJointIndex,
                                                                                                             (float) revoluteJointDefinition.getPositionLowerLimit(),
                                                                                                             (float) revoluteJointDefinition.getPositionUpperLimit());

         bulletMultiBodyRobot.addBtMultiBodyConstraint(multiBodyJointLimitConstraint);
      }
      else if (joint instanceof SimPrismaticJoint)
      {
         PrismaticJointDefinition primaticJointDefinition = (PrismaticJointDefinition) jointDefinition;

         int parentBulletJointIndex;
         if (joint.getPredecessor().getParentJoint() == null)
            parentBulletJointIndex = -1;
         else
            parentBulletJointIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getPredecessor().getParentJoint().getName());

         Vector3 jointAxis = new Vector3();
         BulletTools.toBullet(primaticJointDefinition.getAxis(), jointAxis);

         bulletMultiBodyRobot.getBtMultiBody().setupPrismatic(bulletJointIndex,
                                                              linkMass,
                                                              linkInertiaDiagonal,
                                                              parentBulletJointIndex,
                                                              rotationFromParentBullet,
                                                              jointAxis,
                                                              parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationBullet,
                                                              parentJointAfterFrameToLinkCenterOfMassTranslationBullet,
                                                              disableParentCollision);

         btMultiBodyJointLimitConstraint multiBodyJointLimitConstraint = new btMultiBodyJointLimitConstraint(bulletMultiBodyRobot.getBtMultiBody(),
                                                                                                             bulletJointIndex,
                                                                                                             (float) primaticJointDefinition.getPositionLowerLimit(),
                                                                                                             (float) primaticJointDefinition.getPositionUpperLimit());

         bulletMultiBodyRobot.addBtMultiBodyConstraint(multiBodyJointLimitConstraint);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported joint: " + joint.getClass().getSimpleName());
      }

      return bulletMultiBodyRobot.getBtMultiBody().getLink(bulletJointIndex);
   }

   private static Vector3 decomposeMomentOfInertia(RigidBodyDefinition rigidBodyDefinition)
   {
      Vector3 localInertiaDiagonal = new Vector3();
      if (rigidBodyDefinition.getMomentOfInertia().getM01() == 0 && rigidBodyDefinition.getMomentOfInertia().getM02() == 0
            && rigidBodyDefinition.getMomentOfInertia().getM21() == 0)
      {
         localInertiaDiagonal.x = (float) rigidBodyDefinition.getMomentOfInertia().getM00();
         localInertiaDiagonal.y = (float) rigidBodyDefinition.getMomentOfInertia().getM11();
         localInertiaDiagonal.z = (float) rigidBodyDefinition.getMomentOfInertia().getM22();
      }
      else {
         SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
         
         if (svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
         {
            Matrix3D temp_inertia = new Matrix3D();
            temp_inertia.setToDiagonal(svd.getW().getX(), svd.getW().getY(), svd.getW().getZ());
            RotationMatrix svd_rotation = new RotationMatrix(svd.getU());
            svd_rotation.transform(temp_inertia);

            localInertiaDiagonal.x = (float) temp_inertia.getM00();
            localInertiaDiagonal.y = (float) temp_inertia.getM11();
            localInertiaDiagonal.z = (float) temp_inertia.getM22();
         }
         else
         {
            LogTools.warn("SVD did not decompose", rigidBodyDefinition.getName());
         }
      }
      if (localInertiaDiagonal.x < 0 || localInertiaDiagonal.x > (localInertiaDiagonal.y + localInertiaDiagonal.z) || localInertiaDiagonal.y < 0
            || localInertiaDiagonal.y > (localInertiaDiagonal.x + localInertiaDiagonal.z) || localInertiaDiagonal.z < 0
            || localInertiaDiagonal.z > (localInertiaDiagonal.x + localInertiaDiagonal.y))
      {
         LogTools.warn("Bad inertia tensor properties, setting inertia to zero for link:", rigidBodyDefinition.getName());
         localInertiaDiagonal.x = 0.f;
         localInertiaDiagonal.y = 0.f;
         localInertiaDiagonal.z = 0.f;
      }
      return localInertiaDiagonal;
   }

   private static btMultiBodyLinkCollider createBulletLinkCollider(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                                   List<CollisionShapeDefinition> collisionShapeDefinitions,
                                                                   ReferenceFrame linkCenterOfMassFrame,
                                                                   int bulletJointIndex,
                                                                   String jointName)
   {
      BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider = new BulletMultiBodyLinkCollider(bulletMultiBodyRobot.getBtMultiBody(),
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
         btCollisionShape bulletCollisionShape = BulletTools.createBulletCollisionShape(shapeDefinition);
         bulletCompoundShape.addChildShape(bulletCollisionShapeLocalTransform(shapeDefinition, linkCenterOfMassFrame), bulletCollisionShape);
      }

      bulletMultiBodyLinkCollider.setCollisionShape(bulletCompoundShape);

      bulletMultiBodyRobot.addBulletMuliBodyLinkCollider(bulletMultiBodyLinkCollider);

      return bulletMultiBodyLinkCollider.getBtMultiBodyLinkCollider();
   }

   private static Matrix4 bulletCollisionShapeLocalTransform = new Matrix4();
   private static RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();

   public static Matrix4 bulletCollisionShapeLocalTransform(CollisionShapeDefinition shapeDefinition, ReferenceFrame linkCenterOfMassFrame)
   {
      collisionShapeDefinitionToCenterOfMassFrameTransformEuclid.setAndInvert(linkCenterOfMassFrame.getTransformToParent());
      collisionShapeDefinitionToCenterOfMassFrameTransformEuclid.multiply(new RigidBodyTransform(shapeDefinition.getOriginPose().getRotation(),
                                                                                                 shapeDefinition.getOriginPose().getTranslation()));

      BulletTools.toBullet(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, bulletCollisionShapeLocalTransform);

      return bulletCollisionShapeLocalTransform;
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
