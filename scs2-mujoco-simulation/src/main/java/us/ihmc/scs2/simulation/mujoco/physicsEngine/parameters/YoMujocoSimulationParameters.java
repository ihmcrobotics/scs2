package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * YoVariable-backed mirror of {@link MujocoSimulationParameters}. Drop into a YoRegistry to expose
 * MuJoCo tuning knobs in the SCS2 visualizer.
 */
public class YoMujocoSimulationParameters
{
   private final YoDouble timestep;
   private final YoInteger solverIterations;
   private final YoInteger subSteps;

   public YoMujocoSimulationParameters(String prefix, YoRegistry registry)
   {
      timestep = new YoDouble(prefix + "Timestep", registry);
      solverIterations = new YoInteger(prefix + "SolverIterations", registry);
      subSteps = new YoInteger(prefix + "SubSteps", registry);

      set(MujocoSimulationParameters.defaultMujocoSimulationParameters());
   }

   public void set(MujocoSimulationParameters parameters)
   {
      timestep.set(parameters.getTimestep());
      solverIterations.set(parameters.getSolverIterations());
      subSteps.set(parameters.getSubSteps());
   }

   public double getTimestep()
   {
      return timestep.getDoubleValue();
   }

   public int getSolverIterations()
   {
      return solverIterations.getIntegerValue();
   }

   public int getSubSteps()
   {
      return subSteps.getIntegerValue();
   }
}
