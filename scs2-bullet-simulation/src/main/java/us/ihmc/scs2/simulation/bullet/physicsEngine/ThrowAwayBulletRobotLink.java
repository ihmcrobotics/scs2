package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyJointMotor;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultibodyLink;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class ThrowAwayBulletRobotLink
{
   private final RigidBodyDefinition rigidBodyDefinition;
   private JointBasics parentJoint;
   private RevoluteJoint parentRevoluteJoint;
   private RigidBodyBasics rigidBodyBasics;
   private final YoRegistry yoRegistry;
   private RevoluteJointDefinition parentRevoluteJointDefinition;
   private JointDefinition parentJointDefinition;
   private btMultiBodyLinkCollider bulletMultiBodyLinkCollider;
   private ThrowAwayBulletRobotLinkCollisionSet collisionSet;
   private int jointIndex;
   private int parentJointIndex;
   private final ArrayList<ThrowAwayBulletRobotLink> children = new ArrayList<>();
   private final Matrix4 bulletColliderCenterOfMassTransformToWorldGDX = new Matrix4();
   private final RigidBodyTransform bulletColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
   private MovingReferenceFrame frameAfterParentJoint;
   private btMultibodyLink bulletLink;
   private btMultiBody bulletMultiBody;
   private btMultiBodyJointMotor bulletJointMotor;
   private YoDouble damping;
   private YoDouble bulletJointPosition;
   private YoDouble bulletJointVelocity;
   private YoDouble bulletUserAddedTorque;
   private YoDouble bulletJointTorque;
   private YoDouble bulletLinkAppliedForceX;
   private YoDouble bulletLinkAppliedForceY;
   private YoDouble bulletLinkAppliedForceZ;
   private YoDouble bulletLinkAppliedTorqueX;
   private YoDouble bulletLinkAppliedTorqueY;
   private YoDouble bulletLinkAppliedTorqueZ;

   public ThrowAwayBulletRobotLink(RigidBodyDefinition rigidBodyDefinition,
                                   RigidBodyBasics rigidBodyBasics,
                                   HashMap<String, Integer> jointNameToBulletJointIndexMap,
                                   YoRegistry yoRegistry)
   {
      this.rigidBodyDefinition = rigidBodyDefinition;
      this.rigidBodyBasics = rigidBodyBasics;
      this.yoRegistry = yoRegistry;

      if (rigidBodyDefinition.getParentJoint() != null)
      {
         parentJointDefinition = rigidBodyDefinition.getParentJoint();
         parentJoint = rigidBodyBasics.getParentJoint();
         if (rigidBodyDefinition.getParentJoint() instanceof RevoluteJointDefinition)
         {
            parentRevoluteJointDefinition = (RevoluteJointDefinition) rigidBodyDefinition.getParentJoint();
            parentRevoluteJoint = (RevoluteJoint) parentJoint;
         }

         frameAfterParentJoint = parentJoint.getFrameAfterJoint();
      }

      for (JointDefinition childrenJoint : rigidBodyDefinition.getChildrenJoints())
      {
         for (JointBasics jointBasics : rigidBodyBasics.getChildrenJoints())
         {
            if (jointBasics.getName().equals(childrenJoint.getName()))
            {
               children.add(new ThrowAwayBulletRobotLink(childrenJoint.getSuccessor(), jointBasics.getSuccessor(), jointNameToBulletJointIndexMap, yoRegistry));
            }
         }
      }

      if (parentRevoluteJointDefinition != null)
      {
         jointIndex = jointNameToBulletJointIndexMap.get(parentRevoluteJointDefinition.getName());
         parentJointIndex = jointNameToBulletJointIndexMap.get(parentRevoluteJointDefinition.getParentJoint().getName());

         damping = new YoDouble(parentJoint.getName() + "_damping", yoRegistry);
         bulletJointPosition = new YoDouble(parentJoint.getName() + "_q", yoRegistry);
         bulletJointVelocity = new YoDouble(parentJoint.getName() + "_qd", yoRegistry);
         bulletUserAddedTorque = new YoDouble(parentJoint.getName() + "_btUserAddedTorque", yoRegistry);
         bulletJointTorque = new YoDouble(parentJoint.getName() + "_btJointTorque", yoRegistry);
         bulletLinkAppliedForceX = new YoDouble(parentJoint.getName() + "_btAppliedForceX", yoRegistry);
         bulletLinkAppliedForceY = new YoDouble(parentJoint.getName() + "_btAppliedForceY", yoRegistry);
         bulletLinkAppliedForceZ = new YoDouble(parentJoint.getName() + "_btAppliedForceZ", yoRegistry);
         bulletLinkAppliedTorqueX = new YoDouble(parentJoint.getName() + "_btAppliedTorqueX", yoRegistry);
         bulletLinkAppliedTorqueY = new YoDouble(parentJoint.getName() + "_btAppliedTorqueY", yoRegistry);
         bulletLinkAppliedTorqueZ = new YoDouble(parentJoint.getName() + "_btAppliedTorqueZ", yoRegistry);
      }
      else
      {
         jointIndex = -1;
      }
   }

   public void createBulletJoint(btMultiBody bulletMultiBody)
   {
      this.bulletMultiBody = bulletMultiBody;

      Quaternion rotationFromParentGDX = new Quaternion();
      us.ihmc.euclid.tuple4D.Quaternion euclidRotationFromParent
            = new us.ihmc.euclid.tuple4D.Quaternion(parentRevoluteJointDefinition.getTransformToParent().getRotation());
      euclidRotationFromParent.invert();
      BulletTools.toBullet(euclidRotationFromParent, rotationFromParentGDX);

      RigidBodyTransform parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid = new RigidBodyTransform();
      parentRevoluteJoint.getPredecessor().getBodyFixedFrame().getTransformToDesiredFrame(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid,
                                                                                          parentRevoluteJoint.getFrameBeforeJoint());
      parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.invert();
      Vector3 parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX = new Vector3();
      BulletTools.toBullet(parentLinkCenterOfMassToParentJointBeforeJointFrameTransformEuclid.getTranslation(),
                           parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX);

      RigidBodyTransform parentJointAfterFrameToLinkCenterOfMassTransformEuclid = new RigidBodyTransform();
      parentRevoluteJoint.getFrameAfterJoint().getTransformToDesiredFrame(parentJointAfterFrameToLinkCenterOfMassTransformEuclid,
                                                                          rigidBodyBasics.getBodyFixedFrame());
      parentJointAfterFrameToLinkCenterOfMassTransformEuclid.invert();
      Vector3 parentJointAfterFrameToLinkCenterOfMassTranslationGDX = new Vector3();
      BulletTools.toBullet(parentJointAfterFrameToLinkCenterOfMassTransformEuclid.getTranslation(), parentJointAfterFrameToLinkCenterOfMassTranslationGDX);

      Vector3 baseInertiaDiagonal = new Vector3((float) rigidBodyDefinition.getMomentOfInertia().getM00(),
                                                (float) rigidBodyDefinition.getMomentOfInertia().getM11(),
                                                (float) rigidBodyDefinition.getMomentOfInertia().getM22());
      Vector3 jointAxis = new Vector3();
      BulletTools.toBullet(parentRevoluteJointDefinition.getAxis(), jointAxis);
      boolean disableParentCollision = true;
      bulletMultiBody.setupRevolute(jointIndex,
                                    (float) rigidBodyDefinition.getMass(),
                                    baseInertiaDiagonal,
                                    parentJointIndex,
                                    rotationFromParentGDX,
                                    jointAxis,
                                    parentLinkCenterOfMassToParentJointBeforeJointFrameTranslationGDX,
                                    parentJointAfterFrameToLinkCenterOfMassTranslationGDX,
                                    disableParentCollision);
      bulletLink = bulletMultiBody.getLink(jointIndex);
      bulletLink.setJointDamping((float) parentRevoluteJointDefinition.getDamping()); // Doesn't seem to do anything though
      bulletLink.setJointLowerLimit((float) parentRevoluteJointDefinition.getPositionLowerLimit());
      bulletLink.setJointUpperLimit((float) parentRevoluteJointDefinition.getPositionUpperLimit());
      bulletLink.setJointMaxForce((float) parentRevoluteJointDefinition.getEffortUpperLimit());
      bulletLink.setJointMaxVelocity((float) parentRevoluteJointDefinition.getVelocityUpperLimit());
   }

   public void createBulletCollisionShape(btMultiBody bulletMultiBody, Consumer<btMultiBodyLinkCollider> colliderRegistration)
   {
      collisionSet = new ThrowAwayBulletRobotLinkCollisionSet(rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                              frameAfterParentJoint,
                                                              rigidBodyBasics.getBodyFixedFrame());

      bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultiBody, jointIndex);
      bulletMultiBodyLinkCollider.setCollisionShape(collisionSet.getBulletCompoundShape());
      bulletMultiBodyLinkCollider.setFriction(1.0f);
      colliderRegistration.accept(bulletMultiBodyLinkCollider);
      if (jointIndex >= 0)
         bulletLink.setCollider(bulletMultiBodyLinkCollider);
   }

   public void updateFromMecanoRigidBody()
   {
      rigidBodyBasics.getBodyFixedFrame().getTransformToDesiredFrame(bulletColliderCenterOfMassTransformToWorldEuclid, ReferenceFrame.getWorldFrame());
      BulletTools.toBullet(bulletColliderCenterOfMassTransformToWorldEuclid, bulletColliderCenterOfMassTransformToWorldGDX);
      bulletMultiBodyLinkCollider.setWorldTransform(bulletColliderCenterOfMassTransformToWorldGDX);

      if (parentRevoluteJoint != null)
      {
         bulletMultiBody.setJointPos(jointIndex, (float) parentRevoluteJoint.getQ());
         bulletMultiBody.setJointVel(jointIndex, (float) parentRevoluteJoint.getQd());
      }

      updateFrames();
   }

   public void updateFrames()
   {
      collisionSet.updateFromMecanoRigidBody();
   }

   public void updateJointAngleFromBulletData()
   {
      if (parentRevoluteJoint != null)
      {
         float jointPosition = bulletMultiBody.getJointPos(jointIndex);
         parentRevoluteJoint.setQ(jointPosition);
      }
   }

   public void afterSimulate()
   {
      // https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=36667&hilit=btMultiBody+joint+torque#p36667
      // Assumes fixed time step. TODO: Get time of current step
      bulletJointPosition.set(bulletMultiBody.getJointPos(jointIndex));
      bulletJointVelocity.set(bulletMultiBody.getJointVel(jointIndex));
      bulletUserAddedTorque.set(damping.getValue() * bulletJointVelocity.getValue());
      bulletJointTorque.set(bulletMultiBody.getJointTorque(jointIndex));
      Vector3 linkForce = bulletMultiBody.getLinkForce(jointIndex);
      bulletLinkAppliedForceX.set(linkForce.x);
      bulletLinkAppliedForceY.set(linkForce.y);
      bulletLinkAppliedForceZ.set(linkForce.z);
      Vector3 linkTorque = bulletMultiBody.getLinkTorque(jointIndex);
      bulletLinkAppliedTorqueX.set(linkTorque.x);
      bulletLinkAppliedTorqueY.set(linkTorque.y);
      bulletLinkAppliedTorqueX.set(linkTorque.z);

      //      // TODO: Do in a bullet callback
      //      // TODO: Use GhostObject to reduce required iteration
      //      int numberOfManifolds = bulletPhysicsManager.getMultiBodyDynamicsWorld().getDispatcher().getNumManifolds();
      //      for (int i = 0; i < numberOfManifolds; i++)
      //      {
      //         btPersistentManifold contactManifold = bulletPhysicsManager.getMultiBodyDynamicsWorld().getDispatcher().getManifoldByIndexInternal(i);
      //         int numberOfContacts = contactManifold.getNumContacts();
      //         for (int j = 0; j < numberOfContacts; j++)
      //         {
      //            btManifoldPoint contactPoint = contactManifold.getContactPoint(j);
      //            if (contactPoint.getDistance() < 0.0f)
      //            {
      //               Vector3 pointOnA = new Vector3();
      //               contactPoint.getPositionWorldOnA(pointOnA);
      //               Vector3 pointOnB = new Vector3();
      //               contactPoint.getPositionWorldOnB(pointOnB);
      //               Vector3 normalOnB = new Vector3();
      //               contactPoint.getNormalWorldOnB(normalOnB);
      //            }
      //         }
      //      }

      // Setting stuff
      //      bulletMultiBody.clearForcesAndTorques();
      bulletMultiBody.addJointTorque(jointIndex, (float) bulletUserAddedTorque.getValue());
      //      bulletMultiBody.get
   }

   public boolean isSameLink(RigidBodyDefinition rigidBodyDefinition)
   {
      return this.rigidBodyDefinition.getName().equals(rigidBodyDefinition.getName());
   }

   public boolean isSameLink(RigidBodyBasics rigidBodyBasics)
   {
      return this.rigidBodyDefinition.getName().equals(rigidBodyBasics.getName());
   }

   public void setBulletMultiBodyLinkCollider(btMultiBodyLinkCollider bulletMultiBodyLinkCollider)
   {
      this.bulletMultiBodyLinkCollider = bulletMultiBodyLinkCollider;
   }

   public RigidBodyDefinition getRigidBodyDefinition()
   {
      return rigidBodyDefinition;
   }

   public RigidBodyBasics getRigidBodyBasics()
   {
      return rigidBodyBasics;
   }

   public JointDefinition getParentJointDefinition()
   {
      return parentJointDefinition;
   }

   public JointBasics getParentJoint()
   {
      return parentJoint;
   }

   public RevoluteJoint getParentRevoluteJoint()
   {
      return parentRevoluteJoint;
   }

   public RevoluteJointDefinition getParentRevoluteJointDefinition()
   {
      return parentRevoluteJointDefinition;
   }

   public btMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public ThrowAwayBulletRobotLinkCollisionSet getCollisionSet()
   {
      return collisionSet;
   }

   public ArrayList<ThrowAwayBulletRobotLink> getChildren()
   {
      return children;
   }

   public btMultiBodyJointMotor getBulletJointMotor()
   {
      return bulletJointMotor;
   }

   public YoDouble getDamping()
   {
      return damping;
   }

   public YoDouble getBulletJointPosition()
   {
      return bulletJointPosition;
   }

   public YoDouble getBulletJointVelocity()
   {
      return bulletJointVelocity;
   }

   public YoDouble getBulletUserAddedTorque()
   {
      return bulletUserAddedTorque;
   }

   public YoDouble getBulletJointTorque()
   {
      return bulletJointTorque;
   }

   public YoDouble getBulletLinkAppliedForceX()
   {
      return bulletLinkAppliedForceX;
   }

   public YoDouble getBulletLinkAppliedForceY()
   {
      return bulletLinkAppliedForceY;
   }

   public YoDouble getBulletLinkAppliedForceZ()
   {
      return bulletLinkAppliedForceZ;
   }

   public YoDouble getBulletLinkAppliedTorqueX()
   {
      return bulletLinkAppliedTorqueX;
   }

   public YoDouble getBulletLinkAppliedTorqueY()
   {
      return bulletLinkAppliedTorqueY;
   }

   public YoDouble getBulletLinkAppliedTorqueZ()
   {
      return bulletLinkAppliedTorqueZ;
   }
}
