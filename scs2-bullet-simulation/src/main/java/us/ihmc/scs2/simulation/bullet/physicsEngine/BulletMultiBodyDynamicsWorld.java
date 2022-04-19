package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.CollisionConstants;
import com.badlogic.gdx.physics.bullet.collision.btBroadphaseInterface;
import com.badlogic.gdx.physics.bullet.collision.btCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btDbvtBroadphase;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw;

public class BulletMultiBodyDynamicsWorld
{
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphaseInterface;
   private btMultiBodyConstraintSolver solver;
   private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private btIDebugDraw iDebugDraw;
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> multiBodies = new ArrayList<>();

   public BulletMultiBodyDynamicsWorld()
   {
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphaseInterface = new btDbvtBroadphase();
      solver = new btMultiBodyConstraintSolver();
      iDebugDraw = null;
      multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphaseInterface, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      multiBodyDynamicsWorld.setGravity(gravity);
   }

   public int stepSimulation(double dt)
   {
      int maxSubSteps = 1; // With SCS, we want every tick to get a buffer entry and step through things
      // one step at a time.
      float fixedTimeStep = (float) dt; // SCS has a fixed timestep already so let's just use it
      float timePassedSinceThisWasCalledLast = fixedTimeStep; // We are essentially disabling interpolation here

      return getMultiBodyDynamicsWorld().stepSimulation(timePassedSinceThisWasCalledLast, maxSubSteps, fixedTimeStep);
   }

   public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld()
   {
      return multiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (!multiBodyDynamicsWorld.isDisposed())
      {
         for (BulletTerrainObject terrainObject : terrainObjects)
         {
            terrainObject.getBulletRigidBody().getCollisionShape().dispose();
            terrainObject.getBulletRigidBody().getMotionState().dispose();
            multiBodyDynamicsWorld.removeRigidBody(terrainObject.getBulletRigidBody());
            terrainObject.getBulletRigidBody().dispose();
         }

         for (BulletMultiBodyRobot multiBody : multiBodies)
         {
            for (int i = 0; i < multiBodyDynamicsWorld.getNumConstraints(); i++)
            {
               multiBodyDynamicsWorld.removeConstraint(multiBodyDynamicsWorld.getConstraint(i));
            }

            for (btMultiBodyConstraint multiBodyConstraint : multiBody.getBulletMultiBodyConstrantArray())
            {
               multiBodyDynamicsWorld.removeMultiBodyConstraint(multiBodyConstraint);
               multiBodyConstraint.dispose();
            }

            multiBodyDynamicsWorld.removeMultiBody(multiBody.getBulletMultiBody());
            for (BulletMultiBodyLinkCollider multiBodyLinkCollider : multiBody.getBulletMultiBodyLinkColliderArray())
            {
               multiBodyDynamicsWorld.removeCollisionObject(multiBodyLinkCollider.getMultiBodyLinkCollider());
               multiBodyLinkCollider.getMultiBodyLinkCollider().getCollisionShape().dispose();
               multiBodyLinkCollider.getMultiBodyLinkCollider().dispose();
            }
            for (int i = 0; multiBody.getBulletMultiBody().getNumLinks() < i; i++)
            {
               multiBody.getBulletMultiBody().getLink(i).dispose();
            }
            multiBody.getBulletMultiBody().dispose();
         }
         
         if (iDebugDraw != null)
         {
            iDebugDraw.dispose();
         }
         
         multiBodyDynamicsWorld.dispose();
         solver.dispose();
         broadphaseInterface.dispose();
         collisionDispatcher.dispose();
         collisionConfiguration.dispose();
      }
   }

   public void addMultiBody(BulletMultiBodyRobot bulletMultiBody)
   {
      //add Bullet Multibody to array
      multiBodies.add(bulletMultiBody);
      
      //add Bullet Multibody collisionObjects to multiBodyDynamicsWorld
      for (BulletMultiBodyLinkCollider linkCollider : bulletMultiBody.getBulletMultiBodyLinkColliderArray())
      {
         multiBodyDynamicsWorld.addCollisionObject(linkCollider.getMultiBodyLinkCollider(),
                                                   linkCollider.getCollisionGroup(),
                                                   linkCollider.getCollisionGroupMask());
      }
      
      //add Bullet Multibody constraints to multiBodyDynamicsWorld
      for (btMultiBodyConstraint constraint : bulletMultiBody.getBulletMultiBodyConstrantArray())
      {
         multiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
      }

      //add Bullet Multibody to multiBodyDynamicsWorld
      multiBodyDynamicsWorld.addMultiBody(bulletMultiBody.getBulletMultiBody());
   }

   public void addTerrian(BulletTerrainObject bulletTerrainObject)
   {
      terrainObjects.add(bulletTerrainObject);
      multiBodyDynamicsWorld.addRigidBody(bulletTerrainObject.getBulletRigidBody(),
                                          bulletTerrainObject.getCollisionGroup(),
                                          bulletTerrainObject.getCollisionGroupMask());
      bulletTerrainObject.getBulletRigidBody().setCollisionFlags(bulletTerrainObject.getBulletRigidBody().getCollisionFlags()
            | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
      bulletTerrainObject.getBulletRigidBody().setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
   }
   
   
   public void updateAllMultiBodyParameters (YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      for (BulletMultiBodyRobot bulletMultiBodyRobot : multiBodies)
      {
         bulletMultiBodyRobot.setMultiBodyParameters(bulletMultiBodyParameters);
      }
   }
   
   public void setDebugDrawer(btIDebugDraw iDebugDraw)
   {
      if (!multiBodyDynamicsWorld.isDisposed())
         multiBodyDynamicsWorld.setDebugDrawer(iDebugDraw);
      this.iDebugDraw = iDebugDraw;
   }
   public void debugDrawWorld()
   {
      if (!multiBodyDynamicsWorld.isDisposed())
         multiBodyDynamicsWorld.debugDrawWorld();
   }
}
