package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
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
   private final YoDouble contactSolrefDampRatio;
   private final YoDouble contactSolimpDmin;
   private final YoDouble contactSolimpDmax;
   private final YoInteger noslipIterations;
   private final YoDouble jointArmature;
   private final YoDouble impratio;
   private final YoBoolean useEllipticFrictionCone;
   private final YoDouble frictionSlide;

   public YoMujocoSimulationParameters(String prefix, YoRegistry registry)
   {
      timestep = new YoDouble(prefix + "Timestep", registry);
      solverIterations = new YoInteger(prefix + "SolverIterations", registry);
      subSteps = new YoInteger(prefix + "SubSteps", registry);
      contactSolrefTimeconst = new YoDouble(prefix + "ContactSolrefTimeconst", registry);
      contactSolrefDampRatio = new YoDouble(prefix + "ContactSolrefDampRatio", registry);
      contactSolimpDmin = new YoDouble(prefix + "ContactSolimpDmin", registry);
      contactSolimpDmax = new YoDouble(prefix + "ContactSolimpDmax", registry);
      noslipIterations = new YoInteger(prefix + "NoslipIterations", registry);
      jointArmature = new YoDouble(prefix + "JointArmature", registry);
      impratio = new YoDouble(prefix + "Impratio", registry);
      useEllipticFrictionCone = new YoBoolean(prefix + "UseEllipticFrictionCone", registry);
      frictionSlide = new YoDouble(prefix + "FrictionSlide", registry);

      set(MujocoSimulationParameters.DefaultMujocoSimulationParameters());
   }

   public void set(MujocoSimulationParameters parameters)
   {
      timestep.set(parameters.getTimestep());
      solverIterations.set(parameters.getSolverIterations());
      subSteps.set(parameters.getSubSteps());
      contactSolrefTimeconst.set(parameters.getContactSolrefTimeconst());
      contactSolrefDampRatio.set(parameters.getContactSolrefDampRatio());
      contactSolimpDmin.set(parameters.getContactSolimpDmin());
      contactSolimpDmax.set(parameters.getContactSolimpDmax());
      noslipIterations.set(parameters.getNoslipIterations());
      jointArmature.set(parameters.getJointArmature());
      impratio.set(parameters.getImpratio());
      useEllipticFrictionCone.set(parameters.getUseEllipticFrictionCone());
      frictionSlide.set(parameters.getFrictionSlide());
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

   public double getContactSolrefDampRatio()
   {
      return contactSolrefDampRatio.getDoubleValue();
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

   public double getImpratio()
   {
      return impratio.getDoubleValue();
   }

   public boolean getUseEllipticFrictionCone()
   {
      return useEllipticFrictionCone.getBooleanValue();
   }

   public double getFrictionSlide()
   {
      return frictionSlide.getDoubleValue();
   }

   public MujocoSimulationParameters toPlainParameters()
   {
      MujocoSimulationParameters plain = new MujocoSimulationParameters();
      plain.setTimestep(getTimestep());
      plain.setSolverIterations(getSolverIterations());
      plain.setSubSteps(getSubSteps());
      plain.setContactSolrefTimeconst(getContactSolrefTimeconst());
      plain.setContactSolrefDampRatio(getContactSolrefDampRatio());
      plain.setContactSolimpDmin(getContactSolimpDmin());
      plain.setContactSolimpDmax(getContactSolimpDmax());
      plain.setNoslipIterations(getNoslipIterations());
      plain.setJointArmature(getJointArmature());
      plain.setImpratio(getImpratio());
      plain.setUseEllipticFrictionCone(getUseEllipticFrictionCone());
      plain.setFrictionSlide(getFrictionSlide());
      return plain;
   }
}
