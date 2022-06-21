package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;

public class BulletTerrainObject
{
   private final btMotionState btMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         // Should be always 0, the child shapes are statically placed
      }
   };
   private final btRigidBody btRigidBody;
   private final int collisionGroup = 1; // group 1 is rigid and static bodies
   private final int collisionGroupMask = -1; // Allows interaction with all groups (including custom groups)

   public BulletTerrainObject(float mass, btCollisionShape bulletCompoundCollisionShape)
   {
      Vector3 localInertia = new Vector3();
      bulletCompoundCollisionShape.calculateLocalInertia(mass, localInertia);

      btRigidBody = new btRigidBody(mass, btMotionState, bulletCompoundCollisionShape, localInertia);
      
      btRigidBody.setFriction(1f);
   }

   public btRigidBody getBtRigidBody()
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
