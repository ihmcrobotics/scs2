package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import org.bytedeco.bullet.BulletCollision.btDbvtBroadphase;
import org.bytedeco.bullet.BulletCollision.btDefaultCollisionConfiguration;
import org.bytedeco.bullet.BulletCollision.btCollisionDispatcher;
import org.bytedeco.bullet.BulletCollision.btCollisionObject;
import org.bytedeco.bullet.BulletCollision.btCollisionConfiguration;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraintSolver;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyDynamicsWorld;
import org.bytedeco.bullet.BulletCollision.btBroadphaseInterface;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyConstraint;
import org.bytedeco.bullet.LinearMath.btIDebugDraw;
import org.bytedeco.bullet.LinearMath.btVector3;

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
   private final btVector3 btGravity = new btVector3();

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

   public btMultiBodyDynamicsWorld getBtMultiBodyDynamicsWorld()
   {
      return btMultiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (btMultiBodyDynamicsWorld != null)
      {
         for (BulletTerrainObject bulletTerrainObject : terrainObjects)
         {
            bulletTerrainObject.getBtRigidBody().getCollisionShape().deallocate();
            bulletTerrainObject.getBtRigidBody().getMotionState().deallocate();
            btMultiBodyDynamicsWorld.removeRigidBody(bulletTerrainObject.getBtRigidBody());
            bulletTerrainObject.getBtRigidBody().deallocate();
            ;
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
               bulletMultiBodyConstraint.deallocate();;
            }

            btMultiBodyDynamicsWorld.removeMultiBody(bulletMultiBodyRobot.getBtMultiBody());
            for (BulletMultiBodyLinkCollider multiBodyLinkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
            {
               btMultiBodyDynamicsWorld.removeCollisionObject(multiBodyLinkCollider.getBtMultiBodyLinkCollider());
               multiBodyLinkCollider.getBtMultiBodyLinkCollider().getCollisionShape().deallocate();
               multiBodyLinkCollider.getBtMultiBodyLinkCollider().deallocate();
            }
            for (int i = 0; bulletMultiBodyRobot.getBtMultiBody().getNumLinks() < i; i++)
            {
               bulletMultiBodyRobot.getBtMultiBody().getLink(i).deallocate();
            }
            bulletMultiBodyRobot.getBtMultiBody().deallocate();
         }

         if (btIDebugDraw != null)
         {
            btIDebugDraw.deallocate();
         }

         btMultiBodyDynamicsWorld.deallocate();
         btMultiBodyConstraintSolver.deallocate();
         btBroadphaseInterface.deallocate();
         btCollisionDispatcher.deallocate();
         btCollisionConfiguration.deallocate();
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

   public void setBtDebugDrawer(btIDebugDraw btIDebugDraw)
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
      btMultiBodyDynamicsWorld.getSolverInfo().m_tau((float)globalContactSolverInfoParameters.getTau());
      btMultiBodyDynamicsWorld.getSolverInfo().m_damping((float)globalContactSolverInfoParameters.getDamping());
      btMultiBodyDynamicsWorld.getSolverInfo().m_friction((float)globalContactSolverInfoParameters.getFriction());
      btMultiBodyDynamicsWorld.getSolverInfo().m_timeStep((float)globalContactSolverInfoParameters.getTimeStep());
      btMultiBodyDynamicsWorld.getSolverInfo().m_restitution((float)globalContactSolverInfoParameters.getRestitution());
      btMultiBodyDynamicsWorld.getSolverInfo().m_maxErrorReduction((float)globalContactSolverInfoParameters.getMaxErrorReduction());
      btMultiBodyDynamicsWorld.getSolverInfo().m_numIterations(globalContactSolverInfoParameters.getNumberOfIterations());
      btMultiBodyDynamicsWorld.getSolverInfo().m_erp((float)globalContactSolverInfoParameters.getErrorReductionForNonContactConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().m_erp2((float)globalContactSolverInfoParameters.getErrorReductionForContactConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().m_globalCfm((float)globalContactSolverInfoParameters.getConstraintForceMixingForContactsAndNonContacts());
      btMultiBodyDynamicsWorld.getSolverInfo().m_frictionERP((float)globalContactSolverInfoParameters.getErrorReductionForFrictionConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().m_frictionCFM((float)globalContactSolverInfoParameters.getConstraintForceMixingForFrictionConstraints());
      btMultiBodyDynamicsWorld.getSolverInfo().m_sor((float)globalContactSolverInfoParameters.getSuccessiveOverRelaxationTerm());
      btMultiBodyDynamicsWorld.getSolverInfo().m_splitImpulse(globalContactSolverInfoParameters.getSplitImpulse());
      btMultiBodyDynamicsWorld.getSolverInfo().m_splitImpulsePenetrationThreshold((float)globalContactSolverInfoParameters.getSplitImpulsePenetrationThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().m_splitImpulseTurnErp((float)globalContactSolverInfoParameters.getSplitImpulseTurnErp());
      btMultiBodyDynamicsWorld.getSolverInfo().m_linearSlop((float)globalContactSolverInfoParameters.getLinearSlop());
      btMultiBodyDynamicsWorld.getSolverInfo().m_warmstartingFactor((float)globalContactSolverInfoParameters.getWarmstartingFactor());
      btMultiBodyDynamicsWorld.getSolverInfo().m_solverMode(globalContactSolverInfoParameters.getSolverMode());
      btMultiBodyDynamicsWorld.getSolverInfo().m_restingContactRestitutionThreshold(globalContactSolverInfoParameters.getRestingContactRestitutionThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().m_minimumSolverBatchSize(globalContactSolverInfoParameters.getMinimumSolverBatchSize());
      btMultiBodyDynamicsWorld.getSolverInfo().m_maxGyroscopicForce((float)globalContactSolverInfoParameters.getMaxGyroscopicForce());
      btMultiBodyDynamicsWorld.getSolverInfo().m_singleAxisRollingFrictionThreshold((float)globalContactSolverInfoParameters.getSingleAxisRollingFrictionThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().m_leastSquaresResidualThreshold((float)globalContactSolverInfoParameters.getLeastSquaresResidualThreshold());
      btMultiBodyDynamicsWorld.getSolverInfo().m_restitutionVelocityThreshold((float)globalContactSolverInfoParameters.getRestitutionVelocityThreshold());
   }
}
