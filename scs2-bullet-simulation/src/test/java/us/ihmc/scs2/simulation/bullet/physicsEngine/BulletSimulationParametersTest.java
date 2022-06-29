package us.ihmc.scs2.simulation.bullet.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletSimulationParameters;

public class BulletSimulationParametersTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testDefaultBulletSimulationParameters()
   {
      BulletSimulationParameters simulationParameters = BulletSimulationParameters.defaultBulletSimulateParameters();

      assertSimulationParametersEqual(0.0005, 1, 0.0005, simulationParameters);
   }

   @Test
   public void testConstructor()
   {
      Random random = new Random(23514);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double timeStep = random.nextDouble();
         int maxSubSteps = random.nextInt();
         double fixedTimeStep = random.nextDouble();

         BulletSimulationParameters simulationParameters = new BulletSimulationParameters(timeStep, maxSubSteps, fixedTimeStep);

         assertSimulationParametersEqual(timeStep, maxSubSteps, fixedTimeStep, simulationParameters);
      }
   }

   @Test
   public void testSetters()
   {
      Random random = new Random(748512);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double timeStep = random.nextDouble();
         int maxSubSteps = random.nextInt();
         double fixedTimeStep = random.nextDouble();

         BulletSimulationParameters simulationParameters = new BulletSimulationParameters();

         simulationParameters.setTimeStep(timeStep);
         simulationParameters.setMaxSubSteps(maxSubSteps);
         simulationParameters.setFixedTimeStep(fixedTimeStep);

         assertSimulationParametersEqual(timeStep, maxSubSteps, fixedTimeStep, simulationParameters);
      }
   }

   private static void assertSimulationParametersEqual(double timeStep, int maxSubSteps, double fixedTimeStep, BulletSimulationParameters simulationParameters)
   {
      assertEquals(timeStep, simulationParameters.getTimeStep());
      assertEquals(maxSubSteps, simulationParameters.getMaxSubSteps());
      assertEquals(fixedTimeStep, simulationParameters.getFixedTimeStep());
   }

}
