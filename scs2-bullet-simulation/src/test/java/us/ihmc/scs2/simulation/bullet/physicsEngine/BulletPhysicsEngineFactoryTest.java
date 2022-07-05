package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletSimulationParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.YoBulletSimulationParameters;

public class BulletPhysicsEngineFactoryTest
{
   private static final int ITERATIONS = 100;

   @Test
   public void testNewInstance()
   {
      Random random = new Random(21584);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory());
      assertEquals(simulationSession.getPhysicsEngine().getClass(), BulletPhysicsEngine.class);

      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
      YoBulletMultiBodyJointParameters globalBulletMultiBodyJointParameters = bulletPhysicsEngine.getGlobalBulletMultiBodyJointParameters();
      BulletMultiBodyJointParameters defaultJointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      YoBulletMultiBodyParameters globalBulletMultiBodyParameters = bulletPhysicsEngine.getGlobalBulletMultiBodyParameters();
      BulletMultiBodyParameters defaultParameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      
      YoBulletSimulationParameters globalBulletSimulationParameters = bulletPhysicsEngine.getGlobalSimulationParameters();
      BulletSimulationParameters defaultSimulationParameters = new BulletSimulationParameters();
      
      assetParametersEqual(defaultParameters, defaultJointParameters, defaultSimulationParameters, globalBulletMultiBodyParameters, globalBulletMultiBodyJointParameters, globalBulletSimulationParameters);

      bulletPhysicsEngine.dispose();

      for (int i = 0; i < ITERATIONS; i++)
      {

         BulletMultiBodyParameters parameters = new BulletMultiBodyParameters(random.nextBoolean(),
                                                                              random.nextBoolean(),
                                                                              random.nextBoolean(),
                                                                              random.nextBoolean(),
                                                                              random.nextBoolean(),
                                                                              random.nextDouble(),
                                                                              random.nextDouble(),
                                                                              random.nextDouble(),
                                                                              random.nextDouble());

         BulletMultiBodyJointParameters jointParameters = new BulletMultiBodyJointParameters(random.nextBoolean(),
                                                                                             random.nextDouble(),
                                                                                             random.nextDouble(),
                                                                                             random.nextDouble(),
                                                                                             random.nextDouble(),
                                                                                             random.nextDouble(),
                                                                                             random.nextDouble());

         SimulationSession simulationSessionWithParameters = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters,
                                                                                                                                            jointParameters));

         assertEquals(simulationSession.getPhysicsEngine().getClass(), BulletPhysicsEngine.class);

         BulletPhysicsEngine bulletPhysicsEngineWithParameters = (BulletPhysicsEngine) simulationSessionWithParameters.getPhysicsEngine();
         YoBulletMultiBodyParameters globalBulletMultiBodyParametersFromPhysicsEngine = bulletPhysicsEngineWithParameters.getGlobalBulletMultiBodyParameters();
         YoBulletMultiBodyJointParameters globalBulletMultiBodyJointParametersFromPhysicsEngine = bulletPhysicsEngineWithParameters.getGlobalBulletMultiBodyJointParameters();

         BulletSimulationParameters simulationParameters = new BulletSimulationParameters(random.nextDouble(), random.nextInt(), random.nextDouble());
         bulletPhysicsEngineWithParameters.setGlobalSimulationParameters(simulationParameters);
         YoBulletSimulationParameters globalBulletSimulationParametersFromPhysicsEngine = bulletPhysicsEngineWithParameters.getGlobalSimulationParameters();
         
         assetParametersEqual(parameters,
                              jointParameters,
                              simulationParameters, 
                              globalBulletMultiBodyParametersFromPhysicsEngine,
                              globalBulletMultiBodyJointParametersFromPhysicsEngine,
                              globalBulletSimulationParametersFromPhysicsEngine);

         bulletPhysicsEngineWithParameters.dispose();
      }

   }

   private static void assetParametersEqual(BulletMultiBodyParameters parameters,
                                            BulletMultiBodyJointParameters jointParameters,
                                            BulletSimulationParameters simulationParameters,
                                            YoBulletMultiBodyParameters globalBulletMultiBodyParameters,
                                            YoBulletMultiBodyJointParameters globalBulletMultiBodyJointParameters, 
                                            YoBulletSimulationParameters globalBulletSimulationParameters)
   {
      assertEquals(globalBulletMultiBodyParameters.getAngularDamping(), parameters.getAngularDamping());
      assertEquals(globalBulletMultiBodyParameters.getLinearDamping(), parameters.getLinearDamping());
      assertEquals(globalBulletMultiBodyParameters.getCanSleep(), parameters.getCanSleep());
      assertEquals(globalBulletMultiBodyParameters.getHasSelfCollision(), parameters.getHasSelfCollision());
      assertEquals(globalBulletMultiBodyParameters.getMaxAppliedImpulse(), parameters.getMaxAppliedImpulse());
      assertEquals(globalBulletMultiBodyParameters.getMaxCoordinateVelocity(), parameters.getMaxCoordinateVelocity());
      assertEquals(globalBulletMultiBodyParameters.getUseGlobalVelocities(), parameters.getUseGlobalVelocities());
      assertEquals(globalBulletMultiBodyParameters.getUseGyroTerm(), parameters.getUseGyroTerm());
      assertEquals(globalBulletMultiBodyParameters.getUseRK4Integration(), parameters.getUseRK4Integration());
      assertFalse(globalBulletMultiBodyParameters.getUpdateGlobalMultiBodyParameters());

      assertEquals(globalBulletMultiBodyJointParameters.getJointContactProcessingThreshold(),
                   jointParameters.getJointContactProcessingThreshold());
      assertEquals(globalBulletMultiBodyJointParameters.getJointDisableParentCollision(), jointParameters.getJointDisableParentCollision());
      assertEquals(globalBulletMultiBodyJointParameters.getJointFriction(), jointParameters.getJointFriction());
      assertEquals(globalBulletMultiBodyJointParameters.getJointHitFraction(), jointParameters.getJointHitFraction());
      assertEquals(globalBulletMultiBodyJointParameters.getJointRestitution(), jointParameters.getJointRestitution());
      assertEquals(globalBulletMultiBodyJointParameters.getJointRollingFriction(), jointParameters.getJointRollingFriction());
      assertEquals(globalBulletMultiBodyJointParameters.getJointSpinningFriction(), jointParameters.getJointSpinningFriction());
      assertFalse(globalBulletMultiBodyJointParameters.getUpdateGlobalMultiBodyJointParameters());
      
      assertEquals(globalBulletSimulationParameters.getTimeStamp(), simulationParameters.getTimeStep());
      assertEquals(globalBulletSimulationParameters.getMaxSubSteps(), simulationParameters.getMaxSubSteps());
      assertEquals(globalBulletSimulationParameters.getFixedTimeStep(), simulationParameters.getFixedTimeStep());
   }

}
