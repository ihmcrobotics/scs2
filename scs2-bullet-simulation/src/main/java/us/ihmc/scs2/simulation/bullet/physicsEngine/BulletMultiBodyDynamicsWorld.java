package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import org.bytedeco.bullet.BulletCollision.btCollisionObject;
import org.bytedeco.bullet.BulletCollision.btDbvtBroadphase;
import org.bytedeco.bullet.BulletCollision.btDefaultCollisionConfiguration;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraint;
import org.bytedeco.bullet.LinearMath.btVector3;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletContactSolverInfoParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;

public class BulletMultiBodyDynamicsWorld
{
   private final org.bytedeco.bullet.BulletCollision.btCollisionConfiguration btCollisionConfiguration;
   private final org.bytedeco.bullet.BulletCollision.btCollisionDispatcher btCollisionDispatcher;
   private final org.bytedeco.bullet.BulletCollision.btBroadphaseInterface btBroadphaseInterface;
   private final org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraintSolver btMultiBodyConstraintSolver;
   private final org.bytedeco.bullet.BulletDynamics.btMultiBodyDynamicsWorld btMultiBodyDynamicsWorld;
   private org.bytedeco.bullet.LinearMath.btIDebugDraw btIDebugDraw;
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> multiBodyRobots = new ArrayList<>();
   private final btVector3 btGravity = new btVector3();

   public BulletMultiBodyDynamicsWorld()
   {
      btCollisionConfiguration = new btDefaultCollisionConfiguration();
      btCollisionDispatcher = new org.bytedeco.bullet.BulletCollision.btCollisionDispatcher(btCollisionConfiguration);
      btBroadphaseInterface = new btDbvtBroadphase();
      btMultiBodyConstraintSolver = new org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraintSolver();
      btIDebugDraw = null;
      btMultiBodyDynamicsWorld = new org.bytedeco.bullet.BulletDynamics.btMultiBodyDynamicsWorld(btCollisionDispatcher,
                                                                                                 btBroadphaseInterface,
                                                                                                 btMultiBodyConstraintSolver,
                                                                                                 btCollisionConfiguration);
   }

   public void setGravity(Tuple3DReadOnly gravity)
   {
      btGravity.setValue(gravity.getX32(), gravity.getY32(), gravity.getZ32());
      btMultiBodyDynamicsWorld.setGravity(btGravity);
   }

   public int stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
   }

   public int stepSimulation(float timeStep, int maxSubSteps)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, timeStep);
   }

   public int stepSimulation(float timeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep);
   }

   public org.bytedeco.bullet.BulletDynamics.btMultiBodyDynamicsWorld getBtMultiBodyDynamicsWorld()
   {
      return btMultiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (btMultiBodyDynamicsWorld != null)
      {
//         for (BulletTerrainObject bulletTerrainObject : terrainObjects)
//         {
//            bulletTerrainObject.getBtRigidBody().getCollisionShape().deallocate();
//            bulletTerrainObject.getBtRigidBody().getMotionState().deallocate();
//            btMultiBodyDynamicsWorld.removeRigidBody(bulletTerrainObject.getBtRigidBody());
//            bulletTerrainObject.getBtRigidBody().deallocate();
//            ;
//         }
//
//         for (BulletMultiBodyRobot bulletMultiBodyRobot : multiBodyRobots)
//         {
//            for (int i = 0; i < btMultiBodyDynamicsWorld.getNumConstraints(); i++)
//            {
//               btMultiBodyDynamicsWorld.removeConstraint(btMultiBodyDynamicsWorld.getConstraint(i));
//            }
//
//            for (btMultiBodyConstraint bulletMultiBodyConstraint : bulletMultiBodyRobot.getBtMultiBodyConstraintArray())
//            {
//               btMultiBodyDynamicsWorld.removeMultiBodyConstraint(bulletMultiBodyConstraint);
//               bulletMultiBodyConstraint.deallocate();;
//            }
//
//            btMultiBodyDynamicsWorld.removeMultiBody(bulletMultiBodyRobot.getBtMultiBody());
//            for (BulletMultiBodyLinkCollider multiBodyLinkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
//            {
//               btMultiBodyDynamicsWorld.removeCollisionObject(multiBodyLinkCollider.getBtMultiBodyLinkCollider());
//               multiBodyLinkCollider.getBtMultiBodyLinkCollider().getCollisionShape().deallocate();
//               multiBodyLinkCollider.getBtMultiBodyLinkCollider().deallocate();
//            }
//            for (int i = 0; bulletMultiBodyRobot.getBtMultiBody().getNumLinks() < i; i++)
//            {
//               bulletMultiBodyRobot.getBtMultiBody().getLink(i).deallocate();
//            }
//            bulletMultiBodyRobot.getBtMultiBody().deallocate();
//         }
//
//         if (btIDebugDraw != null)
//         {
//            btIDebugDraw.deallocate();
//         }
//
//         btMultiBodyDynamicsWorld.deallocate();
//         btMultiBodyConstraintSolver.deallocate();
//         btBroadphaseInterface.deallocate();
//         btCollisionDispatcher.deallocate();
//         btCollisionConfiguration.deallocate();
      }
   }

   public void addBulletMultiBodyRobot(BulletMultiBodyRobot bulletMultiBodyRobot)
   {
      // add Bullet Multibody to array
      multiBodyRobots.add(bulletMultiBodyRobot);

      // add Bullet Multibody collisionObjects to multiBodyDynamicsWorld
      for (BulletMultiBodyLinkCollider linkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
      {
         btMultiBodyDynamicsWorld.addCollisionObject(linkCollider.getBtMultiBodyLinkCollider(),
                                                     linkCollider.getCollisionGroup(),
                                                     linkCollider.getCollisionGroupMask());
      }

      // add Bullet Multibody constraints to multiBodyDynamicsWorld
      for (btMultiBodyConstraint constraint : bulletMultiBodyRobot.getBtMultiBodyConstraintArray())
      {
         btMultiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
      }

      // add Bullet Multibody to multiBodyDynamicsWorld
      btMultiBodyDynamicsWorld.addMultiBody(bulletMultiBodyRobot.getBtMultiBody());
   }

   public void addBulletTerrainObject(BulletTerrainObject bulletTerrainObject)
   {
      terrainObjects.add(bulletTerrainObject);
      btMultiBodyDynamicsWorld.addRigidBody(bulletTerrainObject.getBtRigidBody(),
                                            bulletTerrainObject.getCollisionGroup(),
                                            bulletTerrainObject.getCollisionGroupMask());
      bulletTerrainObject.getBtRigidBody()
                         .setCollisionFlags(bulletTerrainObject.getBtRigidBody().getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
      //bulletTerrainObject.getBtRigidBody().setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
      bulletTerrainObject.getBtRigidBody().setActivationState(4);
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

   public void setBtDebugDrawer(org.bytedeco.bullet.LinearMath.btIDebugDraw btIDebugDraw)
   {
//      if (!btMultiBodyDynamicsWorld.isNull())
//         btMultiBodyDynamicsWorld.setDebugDrawer(btIDebugDraw);
//      this.btIDebugDraw = btIDebugDraw;
   }

   public void debugDrawWorld()
   {
//      if (!btMultiBodyDynamicsWorld.isNull())
//         btMultiBodyDynamicsWorld.debugDrawWorld();
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
