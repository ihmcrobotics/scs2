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
import org.bytedeco.bullet.LinearMath.btVector3;
import org.bytedeco.bullet.global.BulletCollision;
import org.bytedeco.bullet.BulletDynamics.btContactSolverInfo;
import org.bytedeco.bullet.LinearMath.btIDebugDraw;
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
   private btIDebugDraw btDebugDraw;
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> multiBodyRobots = new ArrayList<>();
   private final btVector3 btGravity = new btVector3();
   private btContactSolverInfo btContactSolverInfo;

   public BulletMultiBodyDynamicsWorld()
   {
      btCollisionConfiguration = new btDefaultCollisionConfiguration();
      btCollisionDispatcher = new btCollisionDispatcher(btCollisionConfiguration);
      btBroadphaseInterface = new btDbvtBroadphase();
      btMultiBodyConstraintSolver = new btMultiBodyConstraintSolver();
      btDebugDraw = null;
      
      btMultiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(btCollisionDispatcher,
                                                              btBroadphaseInterface,
                                                              btMultiBodyConstraintSolver,
                                                              btCollisionConfiguration);
      
      btContactSolverInfo = btMultiBodyDynamicsWorld.getSolverInfo();
   }

   public void setGravity(Tuple3DReadOnly gravity)
   {
      btGravity.setValue(gravity.getX(), gravity.getY(), gravity.getZ());
      btMultiBodyDynamicsWorld.setGravity(btGravity);
   }

   public int stepSimulation(double timeStep, int maxSubSteps, double fixedTimeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
   }

   public int stepSimulation(double timeStep, int maxSubSteps)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, timeStep);
   }

   public int stepSimulation(double timeStep)
   {
      return btMultiBodyDynamicsWorld.stepSimulation(timeStep);
   }

   public btMultiBodyDynamicsWorld getBtMultiBodyDynamicsWorld()
   {
      return btMultiBodyDynamicsWorld;
   }

   public void dispose()
   {
      if (!btMultiBodyDynamicsWorld.isNull())
      {
         for (BulletTerrainObject bulletTerrainObject : terrainObjects)
         {
            bulletTerrainObject.getBtRigidBody().getCollisionShape().deallocate();
            bulletTerrainObject.getBtRigidBody().getMotionState().deallocate();
            btMultiBodyDynamicsWorld.removeRigidBody(bulletTerrainObject.getBtRigidBody());
            bulletTerrainObject.getBtRigidBody().deallocate();
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
               bulletMultiBodyConstraint.deallocate();
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

         if (btDebugDraw != null)
         {
            btDebugDraw.deallocate();
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
      bulletTerrainObject.getBtRigidBody().setCollisionFlags(bulletTerrainObject.getBtRigidBody().getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
      bulletTerrainObject.getBtRigidBody().setActivationState(BulletCollision.DISABLE_DEACTIVATION);
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

   public void setBtDebugDrawer(btIDebugDraw btDebugDraw)
   {
      if (!btMultiBodyDynamicsWorld.isNull())
         btMultiBodyDynamicsWorld.setDebugDrawer(btDebugDraw);
      this.btDebugDraw = btDebugDraw;
   }

   public void debugDrawWorld()
   {
      if (!btMultiBodyDynamicsWorld.isNull())
         btMultiBodyDynamicsWorld.debugDrawWorld();
   }

   public void updateContactSolverInfoParameters(YoBulletContactSolverInfoParameters globalContactSolverInfoParameters)
   {
      btContactSolverInfo.m_tau(globalContactSolverInfoParameters.getTau());
      btContactSolverInfo.m_damping(globalContactSolverInfoParameters.getDamping());
      btContactSolverInfo.m_friction(globalContactSolverInfoParameters.getFriction());
      btContactSolverInfo.m_timeStep(globalContactSolverInfoParameters.getTimeStep());
      btContactSolverInfo.m_restitution(globalContactSolverInfoParameters.getRestitution());
      btContactSolverInfo.m_maxErrorReduction(globalContactSolverInfoParameters.getMaxErrorReduction());
      btContactSolverInfo.m_numIterations(globalContactSolverInfoParameters.getNumberOfIterations());
      btContactSolverInfo.m_erp(globalContactSolverInfoParameters.getErrorReductionForNonContactConstraints());
      btContactSolverInfo.m_erp2(globalContactSolverInfoParameters.getErrorReductionForContactConstraints());
      btContactSolverInfo.m_globalCfm(globalContactSolverInfoParameters.getConstraintForceMixingForContactsAndNonContacts());
      btContactSolverInfo.m_frictionERP(globalContactSolverInfoParameters.getErrorReductionForFrictionConstraints());
      btContactSolverInfo.m_frictionCFM(globalContactSolverInfoParameters.getConstraintForceMixingForFrictionConstraints());
      btContactSolverInfo.m_sor(globalContactSolverInfoParameters.getSuccessiveOverRelaxationTerm());
      btContactSolverInfo.m_splitImpulse(globalContactSolverInfoParameters.getSplitImpulse());
      btContactSolverInfo.m_splitImpulsePenetrationThreshold(globalContactSolverInfoParameters.getSplitImpulsePenetrationThreshold());
      btContactSolverInfo.m_splitImpulseTurnErp(globalContactSolverInfoParameters.getSplitImpulseTurnErp());
      btContactSolverInfo.m_linearSlop(globalContactSolverInfoParameters.getLinearSlop());
      btContactSolverInfo.m_warmstartingFactor(globalContactSolverInfoParameters.getWarmstartingFactor());
      btContactSolverInfo.m_solverMode(globalContactSolverInfoParameters.getSolverMode());
      btContactSolverInfo.m_restingContactRestitutionThreshold(globalContactSolverInfoParameters.getRestingContactRestitutionThreshold());
      btContactSolverInfo.m_minimumSolverBatchSize(globalContactSolverInfoParameters.getMinimumSolverBatchSize());
      btContactSolverInfo.m_maxGyroscopicForce(globalContactSolverInfoParameters.getMaxGyroscopicForce());
      btContactSolverInfo.m_singleAxisRollingFrictionThreshold(globalContactSolverInfoParameters.getSingleAxisRollingFrictionThreshold());
      btContactSolverInfo.m_leastSquaresResidualThreshold(globalContactSolverInfoParameters.getLeastSquaresResidualThreshold());
      btContactSolverInfo.m_restitutionVelocityThreshold(globalContactSolverInfoParameters.getRestitutionVelocityThreshold());
   }
}
