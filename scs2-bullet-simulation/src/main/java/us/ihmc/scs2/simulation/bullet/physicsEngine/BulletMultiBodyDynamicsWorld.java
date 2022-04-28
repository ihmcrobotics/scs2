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
   private btCollisionConfiguration bulletCollisionConfiguration;
   private btCollisionDispatcher bulletCollisionDispatcher;
   private btBroadphaseInterface bulletBroadphaseInterface;
   private btMultiBodyConstraintSolver bulletMultiBodyConstraintSolver;
   private btMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld;
   private btIDebugDraw bulletIDebugDraw;
   private final ArrayList<BulletTerrainObject> bulletTerrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> bulletMultiBodyRobots = new ArrayList<>();

   public BulletMultiBodyDynamicsWorld()
   {
      bulletCollisionConfiguration = new btDefaultCollisionConfiguration();
      bulletCollisionDispatcher = new btCollisionDispatcher(bulletCollisionConfiguration);
      bulletBroadphaseInterface = new btDbvtBroadphase();
      bulletMultiBodyConstraintSolver = new btMultiBodyConstraintSolver();
      bulletIDebugDraw = null;
      bulletMultiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(bulletCollisionDispatcher, bulletBroadphaseInterface, bulletMultiBodyConstraintSolver, bulletCollisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      bulletMultiBodyDynamicsWorld.setGravity(gravity);
   }

   public int stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
   {
      return bulletMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
   }

   public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld()
   {
      return bulletMultiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (!bulletMultiBodyDynamicsWorld.isDisposed())
      {
         for (BulletTerrainObject bulletTerrainObject : bulletTerrainObjects)
         {
            bulletTerrainObject.getBulletRigidBody().getCollisionShape().dispose();
            bulletTerrainObject.getBulletRigidBody().getMotionState().dispose();
            bulletMultiBodyDynamicsWorld.removeRigidBody(bulletTerrainObject.getBulletRigidBody());
            bulletTerrainObject.getBulletRigidBody().dispose();
         }

         for (BulletMultiBodyRobot bulletMultiBodyRobot : bulletMultiBodyRobots)
         {
            for (int i = 0; i < bulletMultiBodyDynamicsWorld.getNumConstraints(); i++)
            {
               bulletMultiBodyDynamicsWorld.removeConstraint(bulletMultiBodyDynamicsWorld.getConstraint(i));
            }

            for (btMultiBodyConstraint bulletMultiBodyConstraint : bulletMultiBodyRobot.getBulletMultiBodyConstrantArray())
            {
               bulletMultiBodyDynamicsWorld.removeMultiBodyConstraint(bulletMultiBodyConstraint);
               bulletMultiBodyConstraint.dispose();
            }

            bulletMultiBodyDynamicsWorld.removeMultiBody(bulletMultiBodyRobot.getBulletMultiBody());
            for (BulletMultiBodyLinkCollider multiBodyLinkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
            {
               bulletMultiBodyDynamicsWorld.removeCollisionObject(multiBodyLinkCollider.getBulletMultiBodyLinkCollider());
               multiBodyLinkCollider.getBulletMultiBodyLinkCollider().getCollisionShape().dispose();
               multiBodyLinkCollider.getBulletMultiBodyLinkCollider().dispose();
            }
            for (int i = 0; bulletMultiBodyRobot.getBulletMultiBody().getNumLinks() < i; i++)
            {
               bulletMultiBodyRobot.getBulletMultiBody().getLink(i).dispose();
            }
            bulletMultiBodyRobot.getBulletMultiBody().dispose();
         }
         
         if (bulletIDebugDraw != null)
         {
            bulletIDebugDraw.dispose();
         }
         
         bulletMultiBodyDynamicsWorld.dispose();
         bulletMultiBodyConstraintSolver.dispose();
         bulletBroadphaseInterface.dispose();
         bulletCollisionDispatcher.dispose();
         bulletCollisionConfiguration.dispose();
      }
   }

   public void addMultiBody(BulletMultiBodyRobot bulletMultiBody)
   {
      //add Bullet Multibody to array
      bulletMultiBodyRobots.add(bulletMultiBody);
      
      //add Bullet Multibody collisionObjects to multiBodyDynamicsWorld
      for (BulletMultiBodyLinkCollider linkCollider : bulletMultiBody.getBulletMultiBodyLinkColliderArray())
      {
         bulletMultiBodyDynamicsWorld.addCollisionObject(linkCollider.getBulletMultiBodyLinkCollider(),
                                                   linkCollider.getCollisionGroup(),
                                                   linkCollider.getCollisionGroupMask());
      }
      
      //add Bullet Multibody constraints to multiBodyDynamicsWorld
      for (btMultiBodyConstraint constraint : bulletMultiBody.getBulletMultiBodyConstrantArray())
      {
         bulletMultiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
      }

      //add Bullet Multibody to multiBodyDynamicsWorld
      bulletMultiBodyDynamicsWorld.addMultiBody(bulletMultiBody.getBulletMultiBody());
   }

   public void addTerrian(BulletTerrainObject bulletTerrainObject)
   {
      bulletTerrainObjects.add(bulletTerrainObject);
      bulletMultiBodyDynamicsWorld.addRigidBody(bulletTerrainObject.getBulletRigidBody(),
                                          bulletTerrainObject.getCollisionGroup(),
                                          bulletTerrainObject.getCollisionGroupMask());
      bulletTerrainObject.getBulletRigidBody().setCollisionFlags(bulletTerrainObject.getBulletRigidBody().getCollisionFlags()
            | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
      bulletTerrainObject.getBulletRigidBody().setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
   }
   
   
   public void updateAllMultiBodyParameters (YoBulletMultiBodyParameters bulletMultiBodyParameters)
   {
      for (BulletMultiBodyRobot bulletMultiBodyRobot : bulletMultiBodyRobots)
      {
         bulletMultiBodyRobot.setMultiBodyParameters(bulletMultiBodyParameters);
      }
   }
   
   public void updateAllMultiBodyJointParameters (YoBulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      for (BulletMultiBodyRobot bulletMultiBodyRobot : bulletMultiBodyRobots)
      {
         bulletMultiBodyRobot.setMultiBodyJointParameters(bulletMultiBodyJointParameters);
      }
   }
   
   public void setDebugDrawer(btIDebugDraw iDebugDraw)
   {
      if (!bulletMultiBodyDynamicsWorld.isDisposed())
         bulletMultiBodyDynamicsWorld.setDebugDrawer(iDebugDraw);
      this.bulletIDebugDraw = iDebugDraw;
   }
   public void debugDrawWorld()
   {
      if (!bulletMultiBodyDynamicsWorld.isDisposed())
         bulletMultiBodyDynamicsWorld.debugDrawWorld();
   }
}
