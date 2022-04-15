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
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

public class BulletMultiBodyRobotFactory
{
   private static RigidBodyDefinition rigidBodyDefinition;
   private static RigidBodyBasics rigidBody;
   private static HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
   private static int linkCountingIndex = 0;

   public static BulletMultiBodyRobot newInstance(Robot robot, YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      int numberOfLinks = countJoints(robot.getRootBody().getChildrenJoints().get(0)) - 1;
      System.out.println(numberOfLinks);
      JointDefinition rootJointDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0);
      //SixDoFJointDefinition rootSixDoFJointDefinition = (SixDoFJointDefinition) rootJointDefinition;
      System.out.println(rootJointDefinition.getSuccessor().getMass());
      float rootBodyMass = (float) rootJointDefinition.getSuccessor().getMass();
      Vector3 rootBodyIntertia = new Vector3((float) rootJointDefinition.getSuccessor().getMomentOfInertia().getM00(),
                                             (float) rootJointDefinition.getSuccessor().getMomentOfInertia().getM11(),
                                             (float) rootJointDefinition.getSuccessor().getMomentOfInertia().getM22());

      BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks,
                                                                           rootBodyMass,
                                                                           rootBodyIntertia,
                                                                           jointNameToBulletJointIndexMap,
                                                                           bulletMultiBodyParameters);

      JointBasics rootJoint = robot.getRootBody().getChildrenJoints().get(0);
      if (!(rootJoint instanceof SimFloatingRootJoint))
         throw new RuntimeException("Expecting a SimFloatingRootJoint, not a " + rootJoint.getClass().getSimpleName());
      
      rigidBody = rootJoint.getSuccessor();
      rigidBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition().getChildrenJoints().get(0).getSuccessor();

      //Create Bullet BaseCollider
      bulletMultiBodyRobot.getBulletMultiBody()
                          .setBaseCollider(createBulletCollider(bulletMultiBodyRobot,
                                                                rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                                rigidBody.getParentJoint().getFrameAfterJoint(),
                                                                rigidBody.getBodyFixedFrame(),
                                                                -1,
                                                                bulletMultiBodyParameters.getJointFriction()));

      //Create Bullet LinkColliders for each child joint
      createLinkColliders(bulletMultiBodyRobot,
                          robot.getRootBody().getChildrenJoints().get(0),
                          rigidBodyDefinition,
                          bulletMultiBodyParameters.getJointFriction());

      //bulletMultiBodyRobot.finalizeMultiDof();

      return bulletMultiBodyRobot;
   }

   private static void createLinkColliders(BulletMultiBodyRobot bulletMultiBodyRobot,
                                           JointBasics joint,
                                           RigidBodyDefinition jointRigidBodyDefinition,
                                           float friction)
   {
      for (JointBasics childJoint : joint.getSuccessor().getChildrenJoints())
      {
         for (JointDefinition childJointDefinition : jointRigidBodyDefinition.getChildrenJoints())
         {
            if (childJoint.getName().equals(childJointDefinition.getName()))
            {
               int linkIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(childJoint.getName());

               btMultibodyLink bulletLink = setupLink(bulletMultiBodyRobot, childJoint, childJointDefinition, linkIndex);
               bulletLink.setCollider(createBulletCollider(bulletMultiBodyRobot,
                                                           childJointDefinition.getSuccessor().getCollisionShapeDefinitions(),
                                                           childJoint.getFrameAfterJoint(),
                                                           childJoint.getSuccessor().getBodyFixedFrame(),
                                                           linkIndex,
                                                           friction));

               createLinkColliders(bulletMultiBodyRobot, childJoint, childJointDefinition.getSuccessor(), friction);
            }
         }
      }
   }

   private static btMultibodyLink setupLink(BulletMultiBodyRobot bulletMultiBodyRobot, JointBasics joint, JointDefinition jointDefinition, int bulletJointIndex)
   {
      btMultibodyLink bulletLink = null;
      Quaternion rotationFromParentGDX = new Quaternion();
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent 
            = new us.ihmc.euclid.tuple4D.Quaternion(jointDefinition.getTransformToParent().getRotation());
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

         int parentBulletJointIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(revoluteJointDefinition.getParentJoint().getName());

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

         bulletLink = bulletMultiBodyRobot.getBulletMultiBody().getLink(bulletJointIndex);
      }

      return bulletLink;
   }

   private static btMultiBodyLinkCollider createBulletCollider(BulletMultiBodyRobot bulletMultiBodyRobot,
                                                               List<CollisionShapeDefinition> collisionShapeDefinitions,
                                                               ReferenceFrame frameAfterParentJoint,
                                                               ReferenceFrame linkCenterOfMassFrame,
                                                               int bulletJointIndex,
                                                               float friction)
   {
      BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider = new BulletMultiBodyLinkCollider(bulletMultiBodyRobot.getBulletMultiBody(), bulletJointIndex);

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
         bulletCompoundShape.addChildShape(getBulletCollisionShapeLocalTransform(shapeDefinition.getOriginPose(), frameAfterParentJoint, linkCenterOfMassFrame),
                                           bulletCollisionShape);
      }

      bulletMultiBodyLinkCollider.setCollisionShape(bulletCompoundShape);
      bulletMultiBodyLinkCollider.setFriction(friction);

      bulletMultiBodyRobot.addBulletMuliBodyLinkCollider(bulletMultiBodyLinkCollider);

      return bulletMultiBodyLinkCollider.getMultiBodyLinkCollider();
   }

   private static ReferenceFrame collisionShapeDefinitionFrame;
   private static RigidBodyTransform collisionShapeDefinitionToCenterOfMassFrameTransformEuclid = new RigidBodyTransform();

   private static Matrix4 getBulletCollisionShapeLocalTransform(YawPitchRollTransformDefinition collisionShapeToFrameAfterParentJoint,
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

   private static btCollisionShape createBulletCollisionShape(CollisionShapeDefinition collisionShapeDefinition)
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
      jointNameToBulletJointIndexMap.put(joint.getName(), linkCountingIndex - 1);
      ++linkCountingIndex;

      int numberOfJoints = 1;
      for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJoints(childrenJoint);
      }
      return numberOfJoints;
   }

}
