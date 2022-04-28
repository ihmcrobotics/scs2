package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class BulletMultiBodyLinkCollider
{
   btMultiBodyLinkCollider bulletMultiBodyLinkCollider;
   String jointName;
   int linkColliderIndex;
   int collisionGroup;
   int collisionGroupMask;
   private final Matrix4 bulletTempConversionMatrix4 = new Matrix4();
   private final Vector3 bulletTempConversionVector3 = new Vector3();
   private final btVector3 linkForce;
   private final btVector3 linkTorque;

   public BulletMultiBodyLinkCollider(btMultiBody bulletMultibody, int index, String jointName)
   {
      createBulletMultiBodyLinkCollider(bulletMultibody, index, jointName, 2, 1 + 2);
      
      linkForce = bulletMultiBodyLinkCollider.getMultiBody().getLink(linkColliderIndex).getAppliedConstraintForce();
      linkTorque = bulletMultiBodyLinkCollider.getMultiBody().getLink(linkColliderIndex).getAppliedConstraintTorque();
   }

   public void createBulletMultiBodyLinkCollider(btMultiBody bulletMultibody, int index, String jointName, int collisionGroup, int collisionGroupMask)
   {
      bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultibody, index);
      this.linkColliderIndex = index;
      this.jointName = jointName;
      this.collisionGroup = collisionGroup;
      this.collisionGroupMask = collisionGroupMask;
   }

   public void setCollisionGroupMask(int collisionGroup, int collisionGroupMask)
   {
      this.collisionGroup = collisionGroup;
      this.collisionGroupMask = collisionGroupMask;
   }

   public void setCollisionShape(btCollisionShape shape)
   {
      bulletMultiBodyLinkCollider.setCollisionShape(shape);
   }

   public void setFriction(double friction)
   {
      bulletMultiBodyLinkCollider.setFriction((float) friction);
   }

   public void setRestitution(double restitution)
   {
      bulletMultiBodyLinkCollider.setRestitution((float) restitution);
   }

   public void setHitFraction(double hitFraction)
   {
      bulletMultiBodyLinkCollider.setHitFraction((float) hitFraction);
   }

   public void setRollingFriction(double rollingFriction)
   {
      bulletMultiBodyLinkCollider.setRollingFriction((float) rollingFriction);
   }

   public void setSpinningFriction(double spinningFriction)
   {
      bulletMultiBodyLinkCollider.setSpinningFriction((float) spinningFriction);
   }

   public void setContactProcessingThreshold(double contactProcessingThreshold)
   {
      bulletMultiBodyLinkCollider.setContactProcessingThreshold((float) contactProcessingThreshold);
   }

   public btMultiBodyLinkCollider getBulletMultiBodyLinkCollider()
   {
      return bulletMultiBodyLinkCollider;
   }

   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   public int getCollisionGroupMask()
   {
      return collisionGroupMask;
   }

   public String getJointName()
   {
      return jointName;
   }

   public void getWorldTransform(RigidBodyTransform jointSuccessorBodyFixedFrameToWorldEuclid)
   {
      bulletMultiBodyLinkCollider.getWorldTransform(bulletTempConversionMatrix4);
      BulletTools.toEuclid(bulletTempConversionMatrix4, jointSuccessorBodyFixedFrameToWorldEuclid);
   }

   public void setWorldTransform(RigidBodyTransform bulletColliderCenterOfMassTransformToWorldEuclid)
   {
      BulletTools.toBullet(bulletColliderCenterOfMassTransformToWorldEuclid, bulletTempConversionMatrix4);
      bulletMultiBodyLinkCollider.setWorldTransform(bulletTempConversionMatrix4);
      
      if (linkColliderIndex == -1)
         bulletMultiBodyLinkCollider.getMultiBody().setBaseWorldTransform(bulletTempConversionMatrix4);
   }

   public void setJointPos(double jointPosition)
   {
      bulletMultiBodyLinkCollider.getMultiBody().setJointPos(linkColliderIndex, (float) jointPosition);
   }

   public void setJointVel(double jointVelocity)
   {
      bulletMultiBodyLinkCollider.getMultiBody().setJointVel(linkColliderIndex, (float) jointVelocity);
   }

   public void addJointTorque(double jointTau)
   {
      bulletMultiBodyLinkCollider.getMultiBody().addJointTorque(linkColliderIndex, (float) jointTau);
   }

   public float getJointPos()
   {
      return bulletMultiBodyLinkCollider.getMultiBody().getJointPos(linkColliderIndex);
   }

   public float getJointVel()
   {
      return bulletMultiBodyLinkCollider.getMultiBody().getJointVel(linkColliderIndex);
   }

   public void getAppliedConstraintForce(Vector3D force)
   {
      
      force.set((double) linkForce.getX(), (double) linkForce.getY(), (double) linkForce.getZ());
   }

   public void getAppliedConstraintTorque(Vector3D torque)
   {
      torque.set((double) linkTorque.getX(), (double) linkTorque.getY(), (double) linkTorque.getZ());
   }

   public void setBaseVel(Vector3DReadOnly linearVelocityEuclid)
   {
      BulletTools.toBullet(linearVelocityEuclid, bulletTempConversionVector3);
      bulletMultiBodyLinkCollider.getMultiBody().setBaseVel(bulletTempConversionVector3);
   }

   public void setBaseOmega(Vector3DReadOnly angularVelocityBulletEuclid)
   {
      BulletTools.toBullet(angularVelocityBulletEuclid, bulletTempConversionVector3);
      bulletMultiBodyLinkCollider.getMultiBody().setBaseOmega(bulletTempConversionVector3);
   }
   
   public void getBaseVel(Vector3D bulletBaseLinearVelocityEuclid)
   {
      BulletTools.toEuclid(bulletMultiBodyLinkCollider.getMultiBody().getBaseVel(), bulletBaseLinearVelocityEuclid);
   }
   
   public void getBaseOmega(Vector3D bulletBaseAngularVelocityEuclid)
   {
      BulletTools.toEuclid(bulletMultiBodyLinkCollider.getMultiBody().getBaseOmega(), bulletBaseAngularVelocityEuclid);
   }
}
