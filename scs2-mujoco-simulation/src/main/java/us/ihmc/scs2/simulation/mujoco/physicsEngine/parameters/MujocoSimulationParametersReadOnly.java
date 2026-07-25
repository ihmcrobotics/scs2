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

   double getTimestep();

   /**
    * When true, MuJoCo filters contact between each body and its parent (adjacent link pairs in the
    * kinematic tree). This prevents overlapping URDF collision meshes at joints from exploding.
    */
   boolean getFilterParentCollisions();
}
