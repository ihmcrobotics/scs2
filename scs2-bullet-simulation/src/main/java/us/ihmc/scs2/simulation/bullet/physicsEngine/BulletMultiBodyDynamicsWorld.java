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
import org.bytedeco.javacpp.Pointer;
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
   private btIDebugDraw btIDebugDraw;
   private final ArrayList<BulletTerrainObject> terrainObjects = new ArrayList<>();
   private final ArrayList<BulletMultiBodyRobot> multiBodyRobots = new ArrayList<>();
   private final btVector3 btGravity = new btVector3();
   private btContactSolverInfo btContactSolverInfo = new btContactSolverInfo();

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
      
      btContactSolverInfo = btMultiBodyDynamicsWorld.getSolverInfo();
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

   public void setBtDebugDrawer(btIDebugDraw btIDebugDraw)
   {
      //TODO: it looks like the setDebugDrawer method might not be working. After setting it, 
      //      I checked btMultiBodyDynamicsWorld.getDebugDrawer() and it is null. However, the btIDebugDraw object
      //      is not null and the methods can be accessed.
      if (!btMultiBodyDynamicsWorld.isNull())
         btMultiBodyDynamicsWorld.setDebugDrawer(btIDebugDraw);
      this.btIDebugDraw = btIDebugDraw;
      
//      System.out.println("Debug Mode is: " + btIDebugDraw.getDebugMode());
//      if (btMultiBodyDynamicsWorld.getDebugDrawer() == null)
//         System.out.println("btMultiBodyDynamicsWorld.getDebugDrawer() is null"); 
   }

   public void debugDrawWorld()
   {
      if (!btMultiBodyDynamicsWorld.isNull())
         btMultiBodyDynamicsWorld.debugDrawWorld();
   }

   public void updateContactSolverInfoParameters(YoBulletContactSolverInfoParameters globalContactSolverInfoParameters)
   {
      btContactSolverInfo.m_tau((float) globalContactSolverInfoParameters.getTau());
      btContactSolverInfo.m_damping((float) globalContactSolverInfoParameters.getDamping());
      btContactSolverInfo.m_friction((float) globalContactSolverInfoParameters.getFriction());
      btContactSolverInfo.m_timeStep((float) globalContactSolverInfoParameters.getTimeStep());
      btContactSolverInfo.m_restitution((float) globalContactSolverInfoParameters.getRestitution());
      btContactSolverInfo.m_maxErrorReduction((float) globalContactSolverInfoParameters.getMaxErrorReduction());
      btContactSolverInfo.m_numIterations(globalContactSolverInfoParameters.getNumberOfIterations());
      btContactSolverInfo.m_erp((float) globalContactSolverInfoParameters.getErrorReductionForNonContactConstraints());
      btContactSolverInfo.m_erp2((float) globalContactSolverInfoParameters.getErrorReductionForContactConstraints());
      btContactSolverInfo.m_globalCfm((float) globalContactSolverInfoParameters.getConstraintForceMixingForContactsAndNonContacts());
      btContactSolverInfo.m_frictionERP((float) globalContactSolverInfoParameters.getErrorReductionForFrictionConstraints());
      btContactSolverInfo.m_frictionCFM((float) globalContactSolverInfoParameters.getConstraintForceMixingForFrictionConstraints());
      btContactSolverInfo.m_sor((float) globalContactSolverInfoParameters.getSuccessiveOverRelaxationTerm());
      btContactSolverInfo.m_splitImpulse(globalContactSolverInfoParameters.getSplitImpulse());
      btContactSolverInfo.m_splitImpulsePenetrationThreshold((float) globalContactSolverInfoParameters.getSplitImpulsePenetrationThreshold());
      btContactSolverInfo.m_splitImpulseTurnErp((float) globalContactSolverInfoParameters.getSplitImpulseTurnErp());
      btContactSolverInfo.m_linearSlop((float) globalContactSolverInfoParameters.getLinearSlop());
      btContactSolverInfo.m_warmstartingFactor((float) globalContactSolverInfoParameters.getWarmstartingFactor());
      btContactSolverInfo.m_solverMode(globalContactSolverInfoParameters.getSolverMode());
      btContactSolverInfo.m_restingContactRestitutionThreshold(globalContactSolverInfoParameters.getRestingContactRestitutionThreshold());
      btContactSolverInfo.m_minimumSolverBatchSize(globalContactSolverInfoParameters.getMinimumSolverBatchSize());
      btContactSolverInfo.m_maxGyroscopicForce((float) globalContactSolverInfoParameters.getMaxGyroscopicForce());
      btContactSolverInfo.m_singleAxisRollingFrictionThreshold((float) globalContactSolverInfoParameters.getSingleAxisRollingFrictionThreshold());
      btContactSolverInfo.m_leastSquaresResidualThreshold((float) globalContactSolverInfoParameters.getLeastSquaresResidualThreshold());
      btContactSolverInfo.m_restitutionVelocityThreshold((float) globalContactSolverInfoParameters.getRestitutionVelocityThreshold());
   }
}
