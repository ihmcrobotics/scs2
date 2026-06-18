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
   private final YoDouble contactSolrefTimeconst;
   private final YoDouble contactSolimpDmin;
   private final YoDouble contactSolimpDmax;
   private final YoInteger noslipIterations;
   private final YoDouble jointArmature;

   public YoMujocoSimulationParameters(String prefix, YoRegistry registry)
   {
      timestep = new YoDouble(prefix + "Timestep", registry);
      solverIterations = new YoInteger(prefix + "SolverIterations", registry);
      subSteps = new YoInteger(prefix + "SubSteps", registry);
      contactSolrefTimeconst = new YoDouble(prefix + "ContactSolrefTimeconst", registry);
      contactSolimpDmin = new YoDouble(prefix + "ContactSolimpDmin", registry);
      contactSolimpDmax = new YoDouble(prefix + "ContactSolimpDmax", registry);
      noslipIterations = new YoInteger(prefix + "NoslipIterations", registry);
      jointArmature = new YoDouble(prefix + "JointArmature", registry);

      set(MujocoSimulationParameters.defaultMujocoSimulationParameters());
   }

   public void set(MujocoSimulationParameters parameters)
   {
      timestep.set(parameters.getTimestep());
      solverIterations.set(parameters.getSolverIterations());
      subSteps.set(parameters.getSubSteps());
      contactSolrefTimeconst.set(parameters.getContactSolrefTimeconst());
      contactSolimpDmin.set(parameters.getContactSolimpDmin());
      contactSolimpDmax.set(parameters.getContactSolimpDmax());
      noslipIterations.set(parameters.getNoslipIterations());
      jointArmature.set(parameters.getJointArmature());
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

   public double getContactSolrefTimeconst()
   {
      return contactSolrefTimeconst.getDoubleValue();
   }

   public double getContactSolimpDmin()
   {
      return contactSolimpDmin.getDoubleValue();
   }

   public double getContactSolimpDmax()
   {
      return contactSolimpDmax.getDoubleValue();
   }

   public int getNoslipIterations()
   {
      return noslipIterations.getIntegerValue();
   }

   public double getJointArmature()
   {
      return jointArmature.getDoubleValue();
   }

   public MujocoSimulationParameters toPlainParameters()
   {
      MujocoSimulationParameters plain = new MujocoSimulationParameters();
      plain.setTimestep(getTimestep());
      plain.setSolverIterations(getSolverIterations());
      plain.setSubSteps(getSubSteps());
      plain.setContactSolrefTimeconst(getContactSolrefTimeconst());
      plain.setContactSolimpDmin(getContactSolimpDmin());
      plain.setContactSolimpDmax(getContactSolimpDmax());
      plain.setNoslipIterations(getNoslipIterations());
      plain.setJointArmature(getJointArmature());
      return plain;
   }
}
