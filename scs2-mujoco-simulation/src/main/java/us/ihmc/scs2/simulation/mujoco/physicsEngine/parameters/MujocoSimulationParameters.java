package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Holds tunable MuJoCo simulation parameters that map onto entries in {@code mjOption}. Applied
 * once at world compile time; runtime updates are out of scope for v1.
 */
public class MujocoSimulationParameters
{
   private double timestep = 0.002;
   private int solverIterations = 100;
   private int subSteps = 1;
   private double contactSolrefTimeconst = 0.005;

   public static MujocoSimulationParameters defaultMujocoSimulationParameters()
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

   public void set(MujocoSimulationParameters other)
   {
      this.timestep = other.timestep;
      this.solverIterations = other.solverIterations;
      this.subSteps = other.subSteps;
      this.contactSolrefTimeconst = other.contactSolrefTimeconst;
   }
}
