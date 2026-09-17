package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import org.bytedeco.bullet.BulletCollision.btCollisionShape;
import org.bytedeco.bullet.BulletCollision.btCompoundShape;
import org.bytedeco.bullet.LinearMath.btDefaultMotionState;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.bytedeco.bullet.BulletDynamics.btRigidBody;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class BulletTerrainObject
{
   private final btRigidBody btRigidBody;
   private final int collisionGroup = 1; // group 1 is rigid and static bodies
   private final int collisionGroupMask = -1; // Allows interaction with all groups (including custom groups)
   private final btCompoundShape btCollisionShape;
   private ArrayList<btCollisionShape> btCollisionShapes = new ArrayList<>();
   private final btVector3 localInertia = new btVector3();
   // We store this as a field so we can check it in a debugger
   private double friction;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   // This is not actually needed, we just create a new one to pass in.
   // The motion state callbacks will also not work until Bytedeco releases 1.5.9
   private final btDefaultMotionState btMotionState = new btDefaultMotionState();

   public BulletTerrainObject(double mass, btCompoundShape bulletCompoundCollisionShape, ArrayList<btCollisionShape> btCollisionShapes)
   {
      bulletCompoundCollisionShape.calculateLocalInertia(mass, localInertia);

      btRigidBody = new btRigidBody(mass, btMotionState, bulletCompoundCollisionShape, localInertia);
      btRigidBody.setFriction(1.0);
      friction = btRigidBody.getFriction();
      btCollisionShape = bulletCompoundCollisionShape;
      this.setBtCollisionShapes(btCollisionShapes);
   }

   public void pullStateFromBullet()
   {
      friction = btRigidBody.getFriction();
      BulletTools.toEuclid(btRigidBody.getCenterOfMassTransform(), transformToWorld);
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

   public btCompoundShape getBtCollisionShape()
   {
      return btCollisionShape;
   }

   public ArrayList<btCollisionShape> getBtCollisionShapes()
   {
      return btCollisionShapes;
   }

   public void setBtCollisionShapes(ArrayList<btCollisionShape> btCollisionShapes)
   {
      this.btCollisionShapes = btCollisionShapes;
   }

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      // We could set this now, but maybe better to check if it comes back correctly from Bullet
      // transformToWorld.set(transformToWorld);
      BulletTools.toBullet(transformToWorld, btMotionState.m_graphicsWorldTrans());
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }
}
