package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * YoVariable-backed mirror of {@link MujocoSimulationParameters}. Drop into a YoRegistry to expose
 * MuJoCo tuning knobs in the SCS2 visualizer.
 */
public class YoMujocoSimulationParameters implements MujocoSimulationParametersBasics
{
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
   private final YoDouble timestep;
   private final YoBoolean filterParentCollisions;

   public YoMujocoSimulationParameters(String prefix, YoRegistry registry)
   {
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
      timestep = new YoDouble(prefix + "Timestep", registry);
      filterParentCollisions = new YoBoolean(prefix + "FilterParentCollisions", registry);

      set(MujocoSimulationParameters.defaultMujocoSimulationParameters());
   }

   @Override
   public int getSolverIterations()
   {
      return solverIterations.getValue();
   }

   @Override
   public void setSolverIterations(int solverIterations)
   {
      this.solverIterations.set(solverIterations);
   }

   @Override
   public int getSubSteps()
   {
      return subSteps.getValue();
   }

   @Override
   public void setSubSteps(int subSteps)
   {
      this.subSteps.set(subSteps);
   }

   @Override
   public double getContactSolrefTimeconst()
   {
      return contactSolrefTimeconst.getValue();
   }

   @Override
   public void setContactSolrefTimeconst(double contactSolrefTimeconst)
   {
      this.contactSolrefTimeconst.set(contactSolrefTimeconst);
   }

   @Override
   public double getContactSolrefDampRatio()
   {
      return contactSolrefDampRatio.getValue();
   }

   @Override
   public void setContactSolrefDampRatio(double contactSolrefDampRatio)
   {
      this.contactSolrefDampRatio.set(contactSolrefDampRatio);
   }

   @Override
   public double getContactSolimpDmin()
   {
      return contactSolimpDmin.getValue();
   }

   @Override
   public void setContactSolimpDmin(double contactSolimpDmin)
   {
      this.contactSolimpDmin.set(contactSolimpDmin);
   }

   @Override
   public double getContactSolimpDmax()
   {
      return contactSolimpDmax.getValue();
   }

   @Override
   public void setContactSolimpDmax(double contactSolimpDmax)
   {
      this.contactSolimpDmax.set(contactSolimpDmax);
   }

   @Override
   public int getNoslipIterations()
   {
      return noslipIterations.getValue();
   }

   @Override
   public void setNoslipIterations(int noslipIterations)
   {
      this.noslipIterations.set(noslipIterations);
   }

   @Override
   public double getJointArmature()
   {
      return jointArmature.getValue();
   }

   @Override
   public void setJointArmature(double jointArmature)
   {
      this.jointArmature.set(jointArmature);
   }

   @Override
   public double getImpratio()
   {
      return impratio.getValue();
   }

   @Override
   public void setImpratio(double impratio)
   {
      this.impratio.set(impratio);
   }

   @Override
   public boolean getUseEllipticFrictionCone()
   {
      return useEllipticFrictionCone.getValue();
   }

   @Override
   public void setUseEllipticFrictionCone(boolean useEllipticFrictionCone)
   {
      this.useEllipticFrictionCone.set(useEllipticFrictionCone);
   }

   @Override
   public double getFrictionSlide()
   {
      return frictionSlide.getValue();
   }

   @Override
   public void setFrictionSlide(double frictionSlide)
   {
      this.frictionSlide.set(frictionSlide);
   }

   @Override
   public double getTimestep()
   {
      return timestep.getValue();
   }

   @Override
   public void setTimestep(double timestep)
   {
      this.timestep.set(timestep);
   }

   @Override
   public boolean getFilterParentCollisions()
   {
      return filterParentCollisions.getValue();
   }

   @Override
   public void setFilterParentCollisions(boolean filterParentCollisions)
   {
      this.filterParentCollisions.set(filterParentCollisions);
   }
}
