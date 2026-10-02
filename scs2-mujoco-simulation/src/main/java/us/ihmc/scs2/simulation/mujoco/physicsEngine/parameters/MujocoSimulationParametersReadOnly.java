package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import java.util.Map;

/**
 * Compile-time seeds, consumed once when the composite MJCF is generated on the first
 * {@code simulate()} — except {@link #getSubSteps()}, which the engine mirrors live in
 * {@code MujocoOptions}. Defaults live in {@link MujocoSimulationParameters}.
 * <p>
 * Names use MuJoCo's vocabulary verbatim, so each term is directly searchable — the relevant pages
 * are <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters">Modeling
 * &gt; Solver parameters</a> (solref/solimp semantics),
 * <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling &gt;
 * Contact parameters</a> (how per-geom values combine per contact pair), and the
 * <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#geom">XML reference, geom
 * element</a>. Component suffixes (timeconst, dmin, midpoint, ...) are the docs' tuple names, not
 * API symbols; each javadoc states the index. Members are grouped SCS2-owned (camelCase) first,
 * then MuJoCo-owned (snake_case).
 * </p>
 * <p>
 * The contact values here also seed {@code MujocoOptions}' runtime override group
 * ({@code o_solref_*}, {@code o_solimp_*}, ...): {@code o_} is MuJoCo's "override" marker,
 * distinguishing the global {@code mjOption} overrides from these per-geom defaults of the same
 * name, so enabling the override starts from the compiled behavior.
 * </p>
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
   // ---------- SCS2-owned ----------

   /** Number of {@code mj_step} calls per SCS2 tick; consumed once at construction to seed the live {@code MujocoOptions} subSteps variable. */
   int getSubSteps();

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
    * When true, each 1-DoF joint's position limits are emitted as MJCF {@code range} so MuJoCo's
    * constraint solver enforces them. MuJoCo emitted no limits at all before this existed, which
    * let a diverging controller wind a joint far past its range instead of hitting a stop.
    * <p>
    * ContactPointBased applies a soft limit stop instead (see {@code RobotOneDoFJointSoftLimitCalculator});
    * the MJCF uses MuJoCo's default {@code solreflimit}, so the stop is comparably soft but not
    * gain-matched. Set false to restore the old unlimited behavior.
    */
   boolean getEnforceJointLimits();


   /**
    * Command latency in seconds between the controller writing a joint's low-level command and
    * MuJoCo applying it, modelling the lag a real drive has between receiving a setpoint and
    * producing torque. 0 means no delay.
    * <p>
    * Applies to every joint the builder could give an actuator, which is all 1-DoF joints. This is
    * command latency, not measurement latency -- MuJoCo's matching {@code sensor_delay} is indexed
    * by sensor and the MJCF builder emits no {@code <sensor>} elements yet.
    * <p>
    * Compile-time: MuJoCo sizes the history buffer that holds the delayed samples when the model
    * compiles, so this cannot be changed on a running simulation.
    */
   double getActuatorDelay();

   /**
    * Number of samples MuJoCo keeps in the history buffer backing the delay, at least 2. MuJoCo
    * rejects a model that sets a delay without one, so this is not optional when the delay is
    * non-zero.
    * <p>
    * It controls how faithful the delay is, and the minimum of 2 is a trap worth knowing about:
    * the samples span the whole delay window, so with 2 of them linear interpolation ramps the
    * command across the entire delay instead of transporting it, and the joint starts responding
    * immediately. Sample spacing is {@code delay / (samples - 1)}, so for a delay that behaves like
    * real transport lag set this to roughly {@code delay / timestep}. The default of 16 covers the
    * usual case of a few milliseconds of latency at a sub-millisecond timestep.
    */
   int getActuatorDelaySamples();

   /** Per-joint overrides of {@link #getActuatorDelay()}, keyed by SCS2 joint name. */
   Map<String, Double> getActuatorDelayByJointName();

   /**
    * Number of pre-allocated per-contact YoVariable slots (penetration, forces, slip flag);
    * 0 disables per-contact readback. Compile-time by nature: the variables must exist before the
    * session buffer is set up.
    */
   int getPerContactDiagnosticsCapacity();

   // ---------- MuJoCo-owned ----------

   /**
    * MJCF default {@code geom/@solref[0]}: contact settling time constant [s]. Keep &ge;
    * 2&middot;timestep. Empirically for closed-loop locomotion 0.005 s is the sweet spot — 0.02
    * (MuJoCo default) is too soft for the controller; 0.002 gives harsh touchdowns.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#reference">Modeling: solref</a>
    */
   double get_solref_timeconst();

   /**
    * MJCF default {@code geom/@solref[1]} (dampratio): contact damping ratio; 1 = critically damped, &lt; 1 bouncy, &gt; 1 sluggish.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#reference">Modeling: solref</a>
    */
   double get_solref_dampratio();

   /**
    * MJCF default {@code geom/@solimp[0]} (dmin): contact hardness at first touch, 0-1; lower = softer initial contact.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#impedance">Modeling: solimp</a>
    */
   double get_solimp_dmin();

   /**
    * MJCF default {@code geom/@solimp[1]} (dmax): contact hardness once penetrated past width, 0-1.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#impedance">Modeling: solimp</a>
    */
   double get_solimp_dmax();

   /**
    * MJCF default {@code geom/@solimp[2]} (width) [m]: penetration depth over which hardness ramps dmin to dmax.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#impedance">Modeling: solimp</a>
    */
   double get_solimp_width();

   /**
    * MJCF default {@code geom/@solimp[3]} (midpoint): hardness ramp inflection point, 0-1 fraction of width.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#impedance">Modeling: solimp</a>
    */
   double get_solimp_midpoint();

   /**
    * MJCF default {@code geom/@solimp[4]} (power): hardness ramp sharpness, &ge; 1 (1 = linear).
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#impedance">Modeling: solimp</a>
    */
   double get_solimp_power();

   /**
    * MJCF default {@code geom/@friction[0]}: sliding friction coefficient.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling: contact parameters</a>
    */
   double get_friction_slide();

   /**
    * MJCF default {@code geom/@friction[1]}: torsional ("spin") friction coefficient. 0.05
    * empirically balances turn-in-place authority against stance-foot yaw wobble.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling: contact parameters</a>
    */
   double get_friction_spin();

   /**
    * MJCF default {@code geom/@friction[2]}: rolling friction coefficient. 0.01 (vs MuJoCo default 1e-4) so spheres settle in finite time.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling: contact parameters</a>
    */
   double get_friction_roll();

   /**
    * MJCF default {@code geom/@condim}: contact dimensionality (1 frictionless, 3 sliding, 4
    * +torsional, 6 +rolling). Default 4: per-corner torsional friction stops yaw-induced lateral
    * foot slip on flat feet (condim=3 wobbles).
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#condim">Computation: condim</a>
    */
   int get_condim();

   /**
    * MJCF default {@code geom/@margin}: contacts are detected at distance &lt; margin [m].
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#margin-and-gap">Computation: margin and gap</a>
    */
   double get_margin();

   /**
    * MJCF default {@code geom/@gap}: contacts with distance &gt; gap are detected but inactive; force onset at margin − gap [m].
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#margin-and-gap">Computation: margin and gap</a>
    */
   double get_gap();

   /** MJCF default {@code joint/@armature}: rotor inertia added to every joint DoF. */
   double get_armature();
}
