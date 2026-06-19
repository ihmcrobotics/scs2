package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Holds tunable MuJoCo simulation parameters that map onto entries in {@code mjOption}. Applied
 * once at world compile time.
 */
public class MujocoSimulationParameters implements MujocoSimulationParametersBasics
{
   private double timestep = 0.002;
   private int solverIterations = 25;
   private int subSteps = 1;
   private double contactSolrefTimeconst = 0.02;
   private double contactSolrefDampRatio = 1.0;
   private double contactSolimpDmin = 0.9;
   private double contactSolimpDmax = 0.99;
   private int noslipIterations = 5;
   private double jointArmature = 0.0;
   private double impratio = 1.0;
   private boolean useEllipticFrictionCone = false;
   private double frictionSlide = 1.0;

   public static MujocoSimulationParameters defaultMujocoSimulationParameters()
   {
      return new MujocoSimulationParameters();
   }

   @Override
   public double getTimestep()
   {
      return timestep;
   }

   @Override
   public void setTimestep(double timestep)
   {
      this.timestep = timestep;
   }

   @Override
   public int getSolverIterations()
   {
      return solverIterations;
   }

   @Override
   public void setSolverIterations(int solverIterations)
   {
      this.solverIterations = solverIterations;
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
   public int getNoslipIterations()
   {
      return noslipIterations;
   }

   @Override
   public void setNoslipIterations(int noslipIterations)
   {
      this.noslipIterations = noslipIterations;
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
   public double getImpratio()
   {
      return impratio;
   }

   @Override
   public void setImpratio(double impratio)
   {
      this.impratio = impratio;
   }

   @Override
   public boolean getUseEllipticFrictionCone()
   {
      return useEllipticFrictionCone;
   }

   @Override
   public void setUseEllipticFrictionCone(boolean useEllipticFrictionCone)
   {
      this.useEllipticFrictionCone = useEllipticFrictionCone;
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
}
