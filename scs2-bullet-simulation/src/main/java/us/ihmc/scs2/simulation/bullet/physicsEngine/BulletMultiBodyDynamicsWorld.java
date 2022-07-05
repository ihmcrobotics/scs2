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

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;

public class BulletMultiBodyDynamicsWorld
{
   private final btCollisionConfiguration btCollisionConfiguration;
   private final btCollisionDispatcher btCollisionDispatcher;
   private final btBroadphaseInterface btBroadphaseInterface;
   private final btMultiBodyConstraintSolver btMultiBodyConstraintSolver;
   private final btMultiBodyDynamicsWorld btMultiBodyDynamicsWorld;
   private btIDebugDraw btIDebugDraw;
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> multiBodyRobots = new ArrayList<>();
   private final Vector3 btGravity = new Vector3();

   public BulletMultiBodyDynamicsWorld()
   {
      btCollisionConfiguration = new btDefaultCollisionConfiguration();
      btCollisionDispatcher = new btCollisionDispatcher(btCollisionConfiguration);
      btBroadphaseInterface = new btDbvtBroadphase();
      btMultiBodyConstraintSolver = new btMultiBodyConstraintSolver();
      btIDebugDraw = null;
      btMultiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(btCollisionDispatcher,
                                                              btBroadphaseInterface,
                                                              btMultiBodyConstraintSolver,
                                                              btCollisionConfiguration);
   }

   public void setGravity(Tuple3DReadOnly gravity)
   {
      btGravity.set(gravity.getX32(), gravity.getY32(), gravity.getZ32());
      btMultiBodyDynamicsWorld.setGravity(btGravity);
   }

   public int stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
   }

   public int stepSimulation(float timeStep, int maxSubSteps)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps);
   }

   public int stepSimulation(float timeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep);
   }

   public btMultiBodyDynamicsWorld getBtMultiBodyDynamicsWorld()
   {
      return btMultiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (!btMultiBodyDynamicsWorld.isDisposed())
      {
         for (BulletTerrainObject bulletTerrainObject : terrainObjects)
         {
            bulletTerrainObject.getBtRigidBody().getCollisionShape().dispose();
            bulletTerrainObject.getBtRigidBody().getMotionState().dispose();
            btMultiBodyDynamicsWorld.removeRigidBody(bulletTerrainObject.getBtRigidBody());
            bulletTerrainObject.getBtRigidBody().dispose();
         }

         for (BulletMultiBodyRobot bulletMultiBodyRobot : multiBodyRobots)
         {
            for (int i = 0; i < btMultiBodyDynamicsWorld.getNumConstraints(); i++)
            {
               btMultiBodyDynamicsWorld.removeConstraint(btMultiBodyDynamicsWorld.getConstraint(i));
            }

            for (btMultiBodyConstraint bulletMultiBodyConstraint : bulletMultiBodyRobot.getBtMultiBodyConstraintArray())
            {
               btMultiBodyDynamicsWorld.removeMultiBodyConstraint(bulletMultiBodyConstraint);
               bulletMultiBodyConstraint.dispose();
            }

            btMultiBodyDynamicsWorld.removeMultiBody(bulletMultiBodyRobot.getBtMultiBody());
            for (BulletMultiBodyLinkCollider multiBodyLinkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
            {
               btMultiBodyDynamicsWorld.removeCollisionObject(multiBodyLinkCollider.getBtMultiBodyLinkCollider());
               multiBodyLinkCollider.getBtMultiBodyLinkCollider().getCollisionShape().dispose();
               multiBodyLinkCollider.getBtMultiBodyLinkCollider().dispose();
            }
            for (int i = 0; bulletMultiBodyRobot.getBtMultiBody().getNumLinks() < i; i++)
            {
               bulletMultiBodyRobot.getBtMultiBody().getLink(i).dispose();
            }
            bulletMultiBodyRobot.getBtMultiBody().dispose();
         }

         if (btIDebugDraw != null)
         {
            btIDebugDraw.dispose();
         }

         btMultiBodyDynamicsWorld.dispose();
         btMultiBodyConstraintSolver.dispose();
         btBroadphaseInterface.dispose();
         btCollisionDispatcher.dispose();
         btCollisionConfiguration.dispose();
      }
   }

   public void addBulletMultiBodyRobot(BulletMultiBodyRobot bulletMultiBodyRobot)
   {
      //add Bullet Multibody to array
      multiBodyRobots.add(bulletMultiBodyRobot);

      //add Bullet Multibody collisionObjects to multiBodyDynamicsWorld
      for (BulletMultiBodyLinkCollider linkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
      {
         btMultiBodyDynamicsWorld.addCollisionObject(linkCollider.getBtMultiBodyLinkCollider(),
                                                     linkCollider.getCollisionGroup(),
                                                     linkCollider.getCollisionGroupMask());
      }

      //add Bullet Multibody constraints to multiBodyDynamicsWorld
      for (btMultiBodyConstraint constraint : bulletMultiBodyRobot.getBtMultiBodyConstraintArray())
      {
         btMultiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
      }

      //add Bullet Multibody to multiBodyDynamicsWorld
      btMultiBodyDynamicsWorld.addMultiBody(bulletMultiBodyRobot.getBtMultiBody());
   }

   public void addBulletTerrainObject(BulletTerrainObject bulletTerrainObject)
   {
      terrainObjects.add(bulletTerrainObject);
      btMultiBodyDynamicsWorld.addRigidBody(bulletTerrainObject.getBtRigidBody(),
                                            bulletTerrainObject.getCollisionGroup(),
                                            bulletTerrainObject.getCollisionGroupMask());
      bulletTerrainObject.getBtRigidBody()
                         .setCollisionFlags(bulletTerrainObject.getBtRigidBody().getCollisionFlags() | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
      bulletTerrainObject.getBtRigidBody().setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
   }

   public void updateAllMultiBodyParameters(YoBulletMultiBodyParameters multiBodyParameters)
   {
      for (BulletMultiBodyRobot bulletMultiBodyRobot : multiBodyRobots)
      {
         bulletMultiBodyRobot.setMultiBodyParameters(multiBodyParameters);
      }
   }

   public void updateAllMultiBodyJointParameters(YoBulletMultiBodyJointParameters multiBodyJointParameters)
   {
      for (BulletMultiBodyRobot bulletMultiBodyRobot : multiBodyRobots)
      {
         bulletMultiBodyRobot.setMultiBodyJointParameters(multiBodyJointParameters);
      }
   }

   public void setBtDebugDrawer(btIDebugDraw btIDebugDraw)
   {
      if (!btMultiBodyDynamicsWorld.isDisposed())
         btMultiBodyDynamicsWorld.setDebugDrawer(btIDebugDraw);
      this.btIDebugDraw = btIDebugDraw;
   }

   public void debugDrawWorld()
   {
      if (!btMultiBodyDynamicsWorld.isDisposed())
         btMultiBodyDynamicsWorld.debugDrawWorld();
   }

   public void updateContactSolverInfoParameters(YoBulletContactSolverInfoParameters globalContactSolverInfoParameters)
   {
      btMultiBodyDynamicsWorld.getSolverInfo().setTau((float)globalContactSolverInfoParameters.getTau());
      btMultiBodyDynamicsWorld.getSolverInfo().setDamping((float)globalContactSolverInfoParameters.getDamping());
      btMultiBodyDynamicsWorld.getSolverInfo().setFriction((float)globalContactSolverInfoParameters.getFriction());
      btMultiBodyDynamicsWorld.getSolverInfo().setTimeStep((float)globalContactSolverInfoParameters.getTimeStep());
      btMultiBodyDynamicsWorld.getSolverInfo().setRestitution((float)globalContactSolverInfoParameters.getRestitution());
      btMultiBodyDynamicsWorld.getSolverInfo().setMaxErrorReduction((float)globalContactSolverInfoParameters.getMaxErrorReduction());
      btMultiBodyDynamicsWorld.getSolverInfo().setNumIterations(globalContactSolverInfoParameters.getNumberOfIterations());
      btMultiBodyDynamicsWorld.getSolverInfo().setErp((float)globalContactSolverInfoParameters.getErrorReductionForNonContactConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().setErp2((float)globalContactSolverInfoParameters.getErrorReductionForContactConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().setGlobalCfm((float)globalContactSolverInfoParameters.getConstraintForceMixingForContactsAndNonContacts());
      btMultiBodyDynamicsWorld.getSolverInfo().setFrictionERP((float)globalContactSolverInfoParameters.getErrorReductionForFrictionConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().setFrictionCFM((float)globalContactSolverInfoParameters.getConstraintForceMixingForFrictionConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().setSor((float)globalContactSolverInfoParameters.getSuccessiveOverRelaxationTerm());
      btMultiBodyDynamicsWorld.getSolverInfo().setSplitImpulse(globalContactSolverInfoParameters.getSplitImpulse());
      btMultiBodyDynamicsWorld.getSolverInfo().setSplitImpulsePenetrationThreshold((float)globalContactSolverInfoParameters.getSplitImpulsePenetrationThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().setSplitImpulseTurnErp((float)globalContactSolverInfoParameters.getSplitImpulseTurnErp());
      btMultiBodyDynamicsWorld.getSolverInfo().setLinearSlop((float)globalContactSolverInfoParameters.getLinearSlop());
      btMultiBodyDynamicsWorld.getSolverInfo().setWarmstartingFactor((float)globalContactSolverInfoParameters.getWarmstartingFactor());
      btMultiBodyDynamicsWorld.getSolverInfo().setSolverMode(globalContactSolverInfoParameters.getSolverMode());
      btMultiBodyDynamicsWorld.getSolverInfo().setRestingContactRestitutionThreshold(globalContactSolverInfoParameters.getRestingContactRestitutionThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().setMinimumSolverBatchSize(globalContactSolverInfoParameters.getMinimumSolverBatchSize());
      btMultiBodyDynamicsWorld.getSolverInfo().setMaxGyroscopicForce((float)globalContactSolverInfoParameters.getMaxGyroscopicForce());
      btMultiBodyDynamicsWorld.getSolverInfo().setSingleAxisRollingFrictionThreshold((float)globalContactSolverInfoParameters.getSingleAxisRollingFrictionThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().setLeastSquaresResidualThreshold((float)globalContactSolverInfoParameters.getLeastSquaresResidualThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().setRestitutionVelocityThreshold((float)globalContactSolverInfoParameters.getRestitutionVelocityThreshold());
   }
}
