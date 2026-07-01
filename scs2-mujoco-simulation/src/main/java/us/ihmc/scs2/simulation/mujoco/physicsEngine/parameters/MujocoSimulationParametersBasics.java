package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Read-write view of the MuJoCo simulation parameters. Implemented by the plain
 * {@link MujocoSimulationParameters} POJO and the {@link YoMujocoSimulationParameters} Yo-backed
 * mirror.
 */
public interface MujocoSimulationParametersBasics extends MujocoSimulationParametersReadOnly
{
   default void set(MujocoSimulationParametersReadOnly other)
   {
      setSolverIterations(other.getSolverIterations());
      setSubSteps(other.getSubSteps());
      setContactSolrefTimeconst(other.getContactSolrefTimeconst());
      setContactSolrefDampRatio(other.getContactSolrefDampRatio());
      setContactSolimpDmin(other.getContactSolimpDmin());
      setContactSolimpDmax(other.getContactSolimpDmax());
      setNoslipIterations(other.getNoslipIterations());
      setJointArmature(other.getJointArmature());
      setImpratio(other.getImpratio());
      setUseEllipticFrictionCone(other.getUseEllipticFrictionCone());
      setFrictionSlide(other.getFrictionSlide());
      setTimestep(other.getTimestep());
   }

   void setSolverIterations(int solverIterations);

   void setSubSteps(int subSteps);

   void setContactSolrefTimeconst(double contactSolrefTimeconst);

   void setContactSolrefDampRatio(double contactSolrefDampRatio);

   void setContactSolimpDmin(double contactSolimpDmin);

   void setContactSolimpDmax(double contactSolimpDmax);

   void setNoslipIterations(int noslipIterations);

   void setJointArmature(double jointArmature);

   void setImpratio(double impratio);

   void setUseEllipticFrictionCone(boolean useEllipticFrictionCone);

   void setFrictionSlide(double frictionSlide);

   void setTimestep(double timestep);
}
