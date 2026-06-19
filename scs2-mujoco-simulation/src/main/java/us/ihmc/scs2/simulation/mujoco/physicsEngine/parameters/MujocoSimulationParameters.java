package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Holds tunable MuJoCo simulation parameters that map onto entries in {@code mjOption}. Applied
 * once at world compile time.
 */
public class MujocoSimulationParameters
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

   public static MujocoSimulationParameters DefaultMujocoSimulationParameters()
   {
      return new MujocoSimulationParameters();
   }

   public double getTimestep()
   {
      return timestep;
   }

   public void setTimestep(double timestep)
   {
      this.timestep = timestep;
   }

   public int getSolverIterations()
   {
      return solverIterations;
   }

   public void setSolverIterations(int solverIterations)
   {
      this.solverIterations = solverIterations;
   }

   public int getSubSteps()
   {
      return subSteps;
   }

   public void setSubSteps(int subSteps)
   {
      this.subSteps = subSteps;
   }

   public double getContactSolrefTimeconst()
   {
      return contactSolrefTimeconst;
   }

   public void setContactSolrefTimeconst(double contactSolrefTimeconst)
   {
      this.contactSolrefTimeconst = contactSolrefTimeconst;
   }

   public double getContactSolrefDampRatio()
   {
      return contactSolrefDampRatio;
   }

   public void setContactSolrefDampRatio(double contactSolrefDampRatio)
   {
      this.contactSolrefDampRatio = contactSolrefDampRatio;
   }

   public double getContactSolimpDmin()
   {
      return contactSolimpDmin;
   }

   public void setContactSolimpDmin(double contactSolimpDmin)
   {
      this.contactSolimpDmin = contactSolimpDmin;
   }

   public double getContactSolimpDmax()
   {
      return contactSolimpDmax;
   }

   public void setContactSolimpDmax(double contactSolimpDmax)
   {
      this.contactSolimpDmax = contactSolimpDmax;
   }

   public int getNoslipIterations()
   {
      return noslipIterations;
   }

   public void setNoslipIterations(int noslipIterations)
   {
      this.noslipIterations = noslipIterations;
   }

   public double getJointArmature()
   {
      return jointArmature;
   }

   public void setJointArmature(double jointArmature)
   {
      this.jointArmature = jointArmature;
   }

   public double getImpratio()
   {
      return impratio;
   }

   public void setImpratio(double impratio)
   {
      this.impratio = impratio;
   }

   public boolean getUseEllipticFrictionCone()
   {
      return useEllipticFrictionCone;
   }

   public void setUseEllipticFrictionCone(boolean useEllipticFrictionCone)
   {
      this.useEllipticFrictionCone = useEllipticFrictionCone;
   }

   public double getFrictionSlide()
   {
      return frictionSlide;
   }

   public void setFrictionSlide(double frictionSlide)
   {
      this.frictionSlide = frictionSlide;
   }

   public void set(MujocoSimulationParameters other)
   {
      this.timestep = other.timestep;
      this.solverIterations = other.solverIterations;
      this.subSteps = other.subSteps;
      this.contactSolrefTimeconst = other.contactSolrefTimeconst;
      this.contactSolrefDampRatio = other.contactSolrefDampRatio;
      this.contactSolimpDmin = other.contactSolimpDmin;
      this.contactSolimpDmax = other.contactSolimpDmax;
      this.noslipIterations = other.noslipIterations;
      this.jointArmature = other.jointArmature;
      this.impratio = other.impratio;
      this.useEllipticFrictionCone = other.useEllipticFrictionCone;
      this.frictionSlide = other.frictionSlide;
   }
}
