package us.ihmc.scs2.simulation.bullet.physicsEngine;

import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.LinearMath.btDefaultMotionState;
import org.bytedeco.bullet.LinearMath.btTransform;
import org.bytedeco.bullet.LinearMath.btVector3;

public class BulletTerrainObject
{
   private final btDefaultMotionState btMotionState = new btDefaultMotionState()
   {
      @Override
      public void setWorldTransform(btTransform transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }

      @Override
      public void getWorldTransform(btTransform transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }
   };
   private final org.bytedeco.bullet.BulletDynamics.btRigidBody btRigidBody;
   private final int collisionGroup = 1; // group 1 is rigid and static bodies
   private final int collisionGroupMask = -1; // Allows interaction with all groups (including custom groups)

   public BulletTerrainObject(float mass, btCollisionShape bulletCompoundCollisionShape)
   {
      btVector3 localInertia = new btVector3();
      bulletCompoundCollisionShape.calculateLocalInertia(mass, localInertia);

      btRigidBody = new org.bytedeco.bullet.BulletDynamics.btRigidBody(mass, btMotionState, bulletCompoundCollisionShape, localInertia);
      
      btRigidBody.setFriction(1f);
   }

   public org.bytedeco.bullet.BulletDynamics.btRigidBody getBtRigidBody()
   {
      return btRigidBody;
   }

   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   public int getCollisionGroupMask()
   {
      return collisionGroupMask;
   }

}
