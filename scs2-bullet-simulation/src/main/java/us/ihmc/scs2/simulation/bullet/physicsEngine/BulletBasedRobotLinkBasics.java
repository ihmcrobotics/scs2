package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class BulletBasedRobotLinkBasics
{
   private final RigidBodyDefinition rigidBodyDefinition;
   private SimRigidBodyBasics simRigidBody;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap;
   private btMultiBodyLinkCollider bulletMultiBodyLinkCollider;
   private BulletBasedRobotLinkCollisionSet collisionSet;
   private int bulletJointIndex;
   private final ArrayList<BulletBasedRobotLinkBasics> children = new ArrayList<>();
   private final Matrix4 bulletColliderCenterOfMassTransformToWorldBullet = new Matrix4();
   private final RigidBodyTransform bulletColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
   private btMultiBody bulletMultiBody;
   private ReferenceFrame frameAfterJoint;

   public BulletBasedRobotLinkBasics(RigidBodyDefinition rigidBodyDefinition,
                                     SimRigidBodyBasics simRigidBody,
                                     HashMap<String, Integer> jointNameToBulletJointIndexMap)
   {
      this.rigidBodyDefinition = rigidBodyDefinition;
      this.simRigidBody = simRigidBody;
      this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
      frameAfterJoint = simRigidBody.getParentJoint().getFrameAfterJoint();
   }

   public void addChildLinks(YoRegistry yoRegistry)
   {
      for (JointBasics childJoint : simRigidBody.getChildrenJoints())
      {
         for (JointDefinition childJointDefinition : rigidBodyDefinition.getChildrenJoints())
         {
            if (childJoint.getName().equals(childJointDefinition.getName()))
            {
               if (childJoint instanceof SimRevoluteJoint)
               {
                  SimRevoluteJoint childSimRevoluteJoint = (SimRevoluteJoint) childJoint;
                  RevoluteJointDefinition childRevoluteJointDefinition = (RevoluteJointDefinition) childJointDefinition;
                  getChildren().add(new BulletBasedRobotLinkRevolute(childRevoluteJointDefinition,
                                                                     childSimRevoluteJoint,
                                                                     jointNameToBulletJointIndexMap,
                                                                     yoRegistry));
               }
               else
               {
                  throw new RuntimeException("Implement joint type: " + childJoint.getClass().getSimpleName());
               }
            }
         }
      }
   }

   public abstract void setup();

   public void createBulletCollisionShape(BulletBasedPhysicsEngine bulletPhysicsManager)
   {
      collisionSet = new BulletBasedRobotLinkCollisionSet(rigidBodyDefinition.getCollisionShapeDefinitions(),
                                                          frameAfterJoint,
                                                          simRigidBody.getBodyFixedFrame());

      bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultiBody, bulletJointIndex);
      bulletMultiBodyLinkCollider.setCollisionShape(collisionSet.getBulletCompoundShape());
      bulletMultiBodyLinkCollider.setFriction(1.0f);
      bulletPhysicsManager.addMultiBodyCollisionShape(bulletMultiBodyLinkCollider);
   }

   public void updateBulletLinkColliderTransformFromMecanoRigidBody()
   {
      simRigidBody.getBodyFixedFrame().getTransformToDesiredFrame(bulletColliderCenterOfMassTransformToWorldEuclid, SimulationSession.DEFAULT_INERTIAL_FRAME);
      BulletTools.toBullet(bulletColliderCenterOfMassTransformToWorldEuclid, bulletColliderCenterOfMassTransformToWorldBullet);
      bulletMultiBodyLinkCollider.setWorldTransform(bulletColliderCenterOfMassTransformToWorldBullet);
   }

   public abstract void copyDataFromSCSToBullet();

   public abstract void copyBulletJointDataToSCS();

   public void setBulletJointIndex(int bulletJointIndex)
   {
      this.bulletJointIndex = bulletJointIndex;
   }

   public void setBulletMultiBody(btMultiBody bulletMultiBody)
   {
      this.bulletMultiBody = bulletMultiBody;
   }

   public ArrayList<BulletBasedRobotLinkBasics> getChildren()
   {
      return children;
   }

   public HashMap<String, Integer> getJointNameToBulletJointIndexMap()
   {
      return jointNameToBulletJointIndexMap;
   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBody;
   }

   public RigidBodyDefinition getRigidBodyDefinition()
   {
      return rigidBodyDefinition;
   }

   public SimRigidBodyBasics getSimRigidBody()
   {
      return simRigidBody;
   }

   public int getBulletJointIndex()
   {
      return bulletJointIndex;
   }

   public btMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public RigidBodyTransform getBulletColliderCenterOfMassTransformToWorldEuclid()
   {
      return bulletColliderCenterOfMassTransformToWorldEuclid;
   }

   public Matrix4 getBulletColliderCenterOfMassTransformToWorldBullet()
   {
      return bulletColliderCenterOfMassTransformToWorldBullet;
   }
}
