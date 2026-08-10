package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Compile-time seeds, consumed once when the composite MJCF is generated on the first
 * {@code simulate()} — except {@link #getSubSteps()}, which the engine mirrors as a live
 * {@code subSteps} YoVariable. Defaults live in {@link MujocoSimulationParameters}.
 * <p>
 * Rule of thumb for where a value belongs: if MuJoCo would need a model recompile to honor a
 * change (per-geom/per-joint attributes, collision structure, allocation sizes), it is a parameter
 * and goes here; if the {@code mjOption} struct accepts it between steps (global solver and
 * contact-override settings), it is an option and belongs in {@link YoMujocoOptions}, where it is
 * live-tunable from the GUI.
 * </p>
 */
public interface MujocoSimulationParametersReadOnly
{
   /** Number of {@code mj_step} calls per SCS2 tick; the MuJoCo timestep is session dt / subSteps. Live (read every tick). */
   int getSubSteps();

   /**
    * MJCF default {@code geom/@solref[0]}: contact settling time constant [s]. Keep &ge;
    * 2&middot;timestep. Empirically for closed-loop locomotion 0.005 s is the sweet spot — 0.02
    * (MuJoCo default) is too soft for the controller; 0.002 gives harsh touchdowns.
    */
   double getContactSolrefTimeconst();

   /** MJCF default {@code geom/@solref[1]}: contact damping ratio; 1 = critically damped (no bounce). */
   double getContactSolrefDampRatio();

   /** MJCF default {@code geom/@solimp[0]} (dmin): impedance at first touch. */
   double getContactSolimpDmin();

   /** MJCF default {@code geom/@solimp[1]} (dmax): impedance once fully penetrated past width. */
   double getContactSolimpDmax();

   /** MJCF default {@code joint/@armature}: rotor inertia added to every joint DoF. */
   double getJointArmature();

   /** MJCF default {@code geom/@friction[0]}: sliding friction coefficient. */
   double getFrictionSlide();

   /**
    * @deprecated Never read: the effective MuJoCo timestep is the session dt divided by
    *             {@link #getSubSteps()}, set every tick in {@code MujocoPhysicsEngine.simulate}.
    */
   @Deprecated
   double getTimestep();

   /**
    * When true, MuJoCo filters contact between each body and its parent (adjacent link pairs in the
    * kinematic tree). This prevents overlapping URDF collision meshes at joints from exploding.
    */
   boolean getFilterParentCollisions();

   /**
    * MJCF default {@code geom/@friction[1]}: torsional ("spin") friction coefficient. 0.05
    * empirically balances turn-in-place authority against stance-foot yaw wobble.
    */
   double getFrictionTorsional();

   /** MJCF default {@code geom/@friction[2]}: rolling friction coefficient. 0.01 (vs MuJoCo default 1e-4) so spheres settle in finite time. */
   double getFrictionRolling();

   /** MJCF default {@code geom/@solimp[2]} (width): softening-zone thickness [m]. */
   double getContactSolimpWidth();

   /** MJCF default {@code geom/@solimp[3]} (midpoint): impedance sigmoid inflection point. */
   double getContactSolimpMidpoint();

   /** MJCF default {@code geom/@solimp[4]} (power): impedance sigmoid sharpness, &ge; 1. */
   double getContactSolimpPower();

   /**
    * MJCF default {@code geom/@condim}: contact dimensionality (1 frictionless, 3 sliding, 4
    * +torsional, 6 +rolling). Default 4: per-corner torsional friction stops yaw-induced lateral
    * foot slip on flat feet (condim=3 wobbles).
    */
   int getCondim();

   /** MJCF default {@code geom/@margin}: contacts are detected at distance &lt; margin [m]. */
   double getContactMargin();

   /** MJCF default {@code geom/@gap}: contacts with distance &gt; gap are detected but inactive; force onset at margin − gap [m]. */
   double getContactGap();

   /**
    * Number of pre-allocated per-contact YoVariable slots (penetration, forces, slip flag);
    * 0 disables per-contact readback. Compile-time by nature: the variables must exist before the
    * session buffer is set up.
    */
   int getPerContactDiagnosticsCapacity();
}
