package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Compile-time MuJoCo simulation seeds; see {@link MujocoSimulationParametersReadOnly}. This class
 * is the single home of the default values.
 */
public class MujocoSimulationParameters implements MujocoSimulationParametersBasics
{
   private int subSteps = 1;
   private double contactSolrefTimeconst = 0.02;
   private double contactSolrefDampRatio = 1.0;
   private double contactSolimpDmin = 0.9;
   private double contactSolimpDmax = 0.99;
   private double jointArmature = 0.0;
   private double frictionSlide = 1.0;
   private double timestep = 0.0;
   private boolean filterParentCollisions = true;
   private double frictionTorsional = 0.05;
   private double frictionRolling = 0.01;
   private double contactSolimpWidth = 0.0007;
   private double contactSolimpMidpoint = 0.5;
   private double contactSolimpPower = 2.0;
   private int condim = 4;
   private double contactMargin = 0.0;
   private double contactGap = 0.0;
   private int perContactDiagnosticsCapacity = 16;

   public static MujocoSimulationParameters defaultMujocoSimulationParameters()
   {
      return new MujocoSimulationParameters();
   }

   @Override
   public int getSubSteps()
   {
      return subSteps;
   }

   @Override
   public void setSubSteps(int subSteps)
   {
      this.subSteps = subSteps;
   }

   @Override
   public double getContactSolrefTimeconst()
   {
      return contactSolrefTimeconst;
   }

   @Override
   public void setContactSolrefTimeconst(double contactSolrefTimeconst)
   {
      this.contactSolrefTimeconst = contactSolrefTimeconst;
   }

   @Override
   public double getContactSolrefDampRatio()
   {
      return contactSolrefDampRatio;
   }

   @Override
   public void setContactSolrefDampRatio(double contactSolrefDampRatio)
   {
      this.contactSolrefDampRatio = contactSolrefDampRatio;
   }

   @Override
   public double getContactSolimpDmin()
   {
      return contactSolimpDmin;
   }

   @Override
   public void setContactSolimpDmin(double contactSolimpDmin)
   {
      this.contactSolimpDmin = contactSolimpDmin;
   }

   @Override
   public double getContactSolimpDmax()
   {
      return contactSolimpDmax;
   }

   @Override
   public void setContactSolimpDmax(double contactSolimpDmax)
   {
      this.contactSolimpDmax = contactSolimpDmax;
   }

   @Override
   public double getJointArmature()
   {
      return jointArmature;
   }

   @Override
   public void setJointArmature(double jointArmature)
   {
      this.jointArmature = jointArmature;
   }

   @Override
   public double getFrictionSlide()
   {
      return frictionSlide;
   }

   @Override
   public void setFrictionSlide(double frictionSlide)
   {
      this.frictionSlide = frictionSlide;
   }

   @Deprecated
   @Override
   public double getTimestep()
   {
      return timestep;
   }

   @Deprecated
   @Override
   public void setTimestep(double timestep)
   {
      this.timestep = timestep;
   }

   @Override
   public boolean getFilterParentCollisions()
   {
      return filterParentCollisions;
   }

   @Override
   public void setFilterParentCollisions(boolean filterParentCollisions)
   {
      this.filterParentCollisions = filterParentCollisions;
   }

   @Override
   public double getFrictionTorsional()
   {
      return frictionTorsional;
   }

   @Override
   public void setFrictionTorsional(double frictionTorsional)
   {
      this.frictionTorsional = frictionTorsional;
   }

   @Override
   public double getFrictionRolling()
   {
      return frictionRolling;
   }

   @Override
   public void setFrictionRolling(double frictionRolling)
   {
      this.frictionRolling = frictionRolling;
   }

   @Override
   public double getContactSolimpWidth()
   {
      return contactSolimpWidth;
   }

   @Override
   public void setContactSolimpWidth(double contactSolimpWidth)
   {
      this.contactSolimpWidth = contactSolimpWidth;
   }

   @Override
   public double getContactSolimpMidpoint()
   {
      return contactSolimpMidpoint;
   }

   @Override
   public void setContactSolimpMidpoint(double contactSolimpMidpoint)
   {
      this.contactSolimpMidpoint = contactSolimpMidpoint;
   }

   @Override
   public double getContactSolimpPower()
   {
      return contactSolimpPower;
   }

   @Override
   public void setContactSolimpPower(double contactSolimpPower)
   {
      this.contactSolimpPower = contactSolimpPower;
   }

   @Override
   public int getCondim()
   {
      return condim;
   }

   @Override
   public void setCondim(int condim)
   {
      this.condim = condim;
   }

   @Override
   public double getContactMargin()
   {
      return contactMargin;
   }

   @Override
   public void setContactMargin(double contactMargin)
   {
      this.contactMargin = contactMargin;
   }

   @Override
   public double getContactGap()
   {
      return contactGap;
   }

   @Override
   public void setContactGap(double contactGap)
   {
      this.contactGap = contactGap;
   }

   @Override
   public int getPerContactDiagnosticsCapacity()
   {
      return perContactDiagnosticsCapacity;
   }

   @Override
   public void setPerContactDiagnosticsCapacity(int perContactDiagnosticsCapacity)
   {
      this.perContactDiagnosticsCapacity = perContactDiagnosticsCapacity;
   }
}
