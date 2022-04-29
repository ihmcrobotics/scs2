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

import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
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
      Vector3 rootBodyIntertia = new Vector3((float) rigidBodyDefinition.getMomentOfInertia().getM00(),
                                             (float) rigidBodyDefinition.getMomentOfInertia().getM11(),
                                             (float) rigidBodyDefinition.getMomentOfInertia().getM22());

      boolean fixedBase = hasBaseCollider ? false : true;
      BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks,
                                                                           rootBodyMass,
                                                                           rootBodyIntertia,
                                                                           fixedBase,
                                                                           bulletMultiBodyParameters.getCanSleep(),
                                                                           jointNameToBulletJointIndexMap);

      //Create BaseCollider
      if (hasBaseCollider)
      {
         bulletMultiBodyRobot.getBtMultiBody()
                             .setBaseCollider(createBulletLinkCollider(bulletMultiBodyRobot,
                                                                       rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                                       rootJoint.getFrameAfterJoint(),
                                                                       rootJoint.getSuccessor().getBodyFixedFrame(),
                                                                       -1,
                                                                       rootJoint.getName()));
         //Create LinkColliders for each robot joint 
         createBulletLinkColliderChildren(bulletMultiBodyRobot, robot, rootJoint, bulletMultiBodyJointParameters.getJointDisableParentCollision());

      }
      else
      {
         for (JointBasics joint : robot.getRootBody().getChildrenJoints())
         {
            int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(joint.getName());
            JointDefinition rootJointDefinition = robot.getRobotDefinition().getJointDefinition(joint.getName());
            btMultibodyLink bulletLink = setupBtMultibodyLink(bulletMultiBodyRobot,
                                                              joint,
                                                              rootJointDefinition,
                                                              linkIndex,
                                                              bulletMultiBodyJointParameters.getJointDisableParentCollision());
            bulletLink.setCollider(createBulletLinkCollider(bulletMultiBodyRobot,
                                                            rootJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                            joint.getFrameAfterJoint(),
                                                            joint.getSuccessor().getBodyFixedFrame(),
                                                            linkIndex,
                                                            joint.getName()));

            //Create LinkColliders for each root joint 
            createBulletLinkColliderChildren(bulletMultiBodyRobot, robot, joint, bulletMultiBodyJointParameters.getJointDisableParentCollision());
         }
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
                                                              childJoint.getFrameAfterJoint(),
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

         bulletMultiBodyRobot.getBtMultiBody().setupRevolute(bulletJointIndex,
                                                             linkMass,
                                                             baseInertiaDiagonal,
                                                             parentBulletJointIndex,
                                                             rotationFromParentGDX,
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
                                                              baseInertiaDiagonal,
                                                              parentBulletJointIndex,
                                                              rotationFromParentGDX,
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

   private static btMultiBodyLinkCollider createBulletLinkCollider(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                                   List<CollisionShapeDefinition> collisionShapeDefinitions,
                                                                   ReferenceFrame frameAfterParentJoint,
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
         bulletCompoundShape.addChildShape(bulletCollisionShapeLocalTransform(shapeDefinition.getOriginPose(), frameAfterParentJoint, linkCenterOfMassFrame),
                                           bulletCollisionShape);
      }

      bulletMultiBodyLinkCollider.setCollisionShape(bulletCompoundShape);

      bulletMultiBodyRobot.addBulletMuliBodyLinkCollider(bulletMultiBodyLinkCollider);

      return bulletMultiBodyLinkCollider.getBtMultiBodyLinkCollider();
   }

   private static Matrix4 bulletCollisionShapeLocalTransform = new Matrix4();
   private static RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();

   private static Matrix4 bulletCollisionShapeLocalTransform(YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint,
                                                             ReferenceFrame frameAfterParentJoint,
                                                             ReferenceFrame linkCenterOfMassFrame)
   {
      //TODO: refactor to avoid creating a frame
      ReferenceFrame collisionShapeDefinitionFrame = new FixedReferenceFrame(frameAfterParentJoint.getName(),
                                                                             frameAfterParentJoint,
                                                                             new RigidBodyTransform(collisionShapeToFrameAfterParentJoint.getRotation(),
                                                                                                    collisionShapeToFrameAfterParentJoint.getTranslation()));

      collisionShapeDefinitionFrame.getTransformToDesiredFrame(collisionShapeDefinitionToCenterOfMassFrameTransformEuclid, linkCenterOfMassFrame);
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
