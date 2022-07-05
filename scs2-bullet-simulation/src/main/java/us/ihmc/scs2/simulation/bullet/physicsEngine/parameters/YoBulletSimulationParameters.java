package us.ihmc.scs2.simulation.bullet.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoBulletSimulationParameters
{
   private YoDouble timeStamp;
   private YoInteger maxSubSteps;
   private YoDouble fixedTimeStep;

   public YoBulletSimulationParameters(String prefix, YoRegistry registry)
   {
      String simulateTimeStamp;
      String simulateMaxSubSteps;
      String simulateFixedTimeStep;

      if (prefix == null || prefix.isEmpty())
      {
         simulateTimeStamp = "TimeStamp";
         simulateMaxSubSteps = "MaxSubSteps";
         simulateFixedTimeStep = "FixedTimeStep";
      }
      else
      {
         simulateTimeStamp = prefix + "TimeStamp";
         simulateMaxSubSteps = prefix + "MaxSubSteps";
         simulateFixedTimeStep = prefix + "FixedTimeStep";
      }

      timeStamp = new YoDouble(simulateTimeStamp, registry);
      maxSubSteps = new YoInteger(simulateMaxSubSteps, registry);
      fixedTimeStep = new YoDouble(simulateFixedTimeStep, registry);
   }

   public void set(BulletSimulationParameters parameters)
   {
      setTimeStamp(parameters.getTimeStep());
      setMaxSubSteps(parameters.getMaxSubSteps());
      setFixedTimeStep(parameters.getFixedTimeStep());
   }

   public void setTimeStamp(Double timeStamp)
   {
      this.timeStamp.set(timeStamp);
   }

   public void setMaxSubSteps(int maxSubSteps)
   {
      this.maxSubSteps.set(maxSubSteps);
   }

   public void setFixedTimeStep(double fixedTimeStep)
   {
      this.fixedTimeStep.set(fixedTimeStep);
   }

   public double getTimeStamp()
   {
      return timeStamp.getValue();
   }

   public int getMaxSubSteps()
   {
      return maxSubSteps.getValue();
   }

   public double getFixedTimeStep()
   {
      return fixedTimeStep.getValue();
   }
}
