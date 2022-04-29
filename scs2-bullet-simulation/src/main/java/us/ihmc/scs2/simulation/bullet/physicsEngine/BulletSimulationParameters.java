package us.ihmc.scs2.simulation.bullet.physicsEngine;

public class BulletSimulationParameters
{
   private double timeStep;
   private int maxSubSteps;
   private double fixedTimeStep;

   public static BulletSimulationParameters defaultBulletSimulateParameters()
   {
      BulletSimulationParameters bulletSimulationParameters = new BulletSimulationParameters();
      bulletSimulationParameters.setTimeStep(0.0005);
      bulletSimulationParameters.setMaxSubSteps(1);
      bulletSimulationParameters.setFixedTimeStep(0.0005);
      return bulletSimulationParameters;
   }

   public BulletSimulationParameters()
   {
   }

   public BulletSimulationParameters(double timeStep, int maxSubSteps, double fixedTimeStep)
   {
      this.timeStep = timeStep;
      this.maxSubSteps = maxSubSteps;
      this.fixedTimeStep = fixedTimeStep;
   }

   public void setTimeStep(double timeStep)
   {
      this.timeStep = timeStep;
   }

   public void setMaxSubSteps(int maxSubSteps)
   {
      this.maxSubSteps = maxSubSteps;
   }

   public void setFixedTimeStep(double fixedTimeStep)
   {
      this.fixedTimeStep = fixedTimeStep;
   }

   public double getTimeStep()
   {
      return timeStep;
   }

   public int getMaxSubSteps()
   {
      return maxSubSteps;
   }

   public double getFixedTimeStep()
   {
      return fixedTimeStep;
   }
}
