package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Read-only view of the MuJoCo simulation parameters. Consumers (the physics engine and MJCF
 * builder) should depend on this interface; producers/mutators use {@link MujocoSimulationParametersBasics}.
 */
public interface MujocoSimulationParametersReadOnly
{
   int getSolverIterations();

   int getSubSteps();

   double getContactSolrefTimeconst();

   double getContactSolrefDampRatio();

   double getContactSolimpDmin();

   double getContactSolimpDmax();

   int getNoslipIterations();

   double getJointArmature();

   double getImpratio();

   boolean getUseEllipticFrictionCone();

   double getFrictionSlide();
}
