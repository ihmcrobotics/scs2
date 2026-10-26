package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import java.util.Map;

/**
 * Compile-time seeds, consumed once when the composite MJCF is generated on the first
 * {@code simulate()} -- except {@link #getSubSteps()}, which the engine mirrors live in
 * {@code MujocoOptions}. Defaults live in {@link MujocoSimulationParameters}.
 * <p>
 * Names use MuJoCo's vocabulary verbatim, so each term is directly searchable -- the relevant pages
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
    * When true, each 1-DoF joint's position limits are emitted as MJCF {@code range}, with the end
    * stop's stiffness from {@link #get_solreflimit_timeconst()}. ContactPointBased's soft limit stop
    * runs only under that engine, so the two mechanisms never both act on a joint.
    */
   boolean getEnforceJointLimits();

   /**
    * Command latency [s] between the controller writing a joint's low-level command and MuJoCo
    * applying it; 0 for none. Command latency only -- MuJoCo's {@code sensor_delay} is per sensor
    * and no {@code <sensor>} elements are emitted. Compile-time: it sizes a history buffer.
    */
   double getActuatorDelay();

   /**
    * History samples backing the delay, at least 2. The samples span the whole delay window, so at
    * the minimum of 2 the command is interpolated across the delay rather than transported through
    * it and the joint responds immediately; spacing is {@code delay / (samples - 1)}, so set this
    * near {@code delay / timestep} for true transport lag.
    */
   int getActuatorDelaySamples();

   /** Per-joint overrides of {@link #getActuatorDelay()}, keyed by SCS2 joint name. */
   Map<String, Double> getActuatorDelayByJointName();

   /**
    * Joints driven by a {@code <dcmotor>} instead of the affine servo, keyed by SCS2 joint name; a
    * joint absent from the map keeps the servo. Compile-time: the actuator element and its
    * activation states are fixed at compile, though the gains inside it stay live.
    */
   Map<String, MujocoMotorParameters> getMotorByJointName();

   /**
    * MJCF {@code joint/@armature} per joint, keyed by SCS2 joint name: the drive's rotor inertia
    * reflected through its gearing, which every joint has a different value of. A joint absent from
    * the map gets none. Compile-time: armature enters the mass matrix when MuJoCo compiles.
    */
   Map<String, Double> getArmatureByJointName();

   /**
    * Number of pre-allocated per-contact YoVariable slots (penetration, forces, slip flag);
    * 0 disables per-contact readback. Compile-time by nature: the variables must exist before the
    * session buffer is set up.
    */
   int getPerContactDiagnosticsCapacity();

   // ---------- MuJoCo-owned ----------

   /**
    * MJCF {@code joint/@solreflimit[0]}: settling time constant [s] of the JOINT LIMIT constraint,
    * i.e. the mechanical end stop, as opposed to {@link #get_solref_timeconst()} for ground
    * contact. Keep &ge; 2&middot;timestep. MuJoCo's default is 0.02.
    * <p>
    * The positive {@code (timeconst, dampratio)} form is normalised by the constraint inertia
    * internally, so one time constant means the same settling on any joint. Do not convert a
    * torque-space gain such as {@code getKpSoftLimitStop()} into the negative
    * {@code (-stiffness, -damping)} form instead: MuJoCo's reference is an ACCELERATION, so that
    * conversion needs the joint's pose-dependent inertia, which a compile-time attribute cannot have.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#reference">Modeling: solref</a>
    */
   double get_solreflimit_timeconst();

   /**
    * MJCF {@code joint/@solreflimit[1]}: damping ratio of the joint limit constraint. 1 is
    * critically damped and is MuJoCo's default; below 1 the joint bounces off its own end stop.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#reference">Modeling: solref</a>
    */
   double get_solreflimit_dampratio();

   /**
    * MJCF default {@code geom/@solref[0]}: contact settling time constant [s]. Keep &ge;
    * 2&middot;timestep. Empirically for closed-loop locomotion 0.005 s is the sweet spot -- 0.02
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
    * MJCF default {@code geom/@solimp[2]} (width) [m]: penetration depth over which hardness ramps
    * dmin to dmax. MuJoCo's default, 1e-3.
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
    * MJCF default {@code geom/@friction[1]}: torsional ("spin") friction, in METRES -- it is the
    * contact patch radius the solver assumes. MuJoCo's tiny default suits contacts whose patch is
    * NOT resolved into separate points (a sphere, a capsule); a box sole is resolved into corner
    * contacts that already resist yaw through their lever arms, so a coefficient on top
    * double-counts. Below the geometry's own corner lever arm it changes nothing; above it the body
    * resists yaw by a fixed amount regardless of how it is actually supported.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling: contact parameters</a>
    */
   double get_friction_spin();

   /**
    * MJCF default {@code geom/@friction[2]}: rolling friction, in metres, like
    * {@link #get_friction_spin()}. Inert below condim 6, and raising it resists foot roll-over at
    * heel-strike and toe-off, which is rarely what a walking robot wants.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters">Modeling: contact parameters</a>
    */
   double get_friction_roll();

   /**
    * MJCF default {@code geom/@condim}: contact dimensionality (1 frictionless, 3 sliding, 4
    * +torsional, 6 +rolling). MuJoCo's default is 3 and so is this one.
    * <p>
    * 3 suits box feet structurally: MuJoCo resolves a box sole against a plane or box into four
    * corner contacts which already carry the whole wrench, so condim 4 adds a torsional friction
    * cone per corner for a patch the geometry has resolved. Raise it to 4 for geometry that really
    * contacts at one point -- a lone sphere or capsule -- and size {@code friction_spin} as that
    * sole's true radius. See {@link #get_friction_spin()}.
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#condim">Computation: condim</a>
    */
   int get_condim();

   /**
    * MJCF default {@code geom/@margin}: contacts are detected at distance &lt; margin [m].
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#margin-and-gap">Computation: margin and gap</a>
    */
   double get_margin();

   /**
    * MJCF default {@code geom/@gap}: contacts with distance &gt; gap are detected but inactive; force onset at margin - gap [m].
    * @see <a href="https://mujoco.readthedocs.io/en/stable/computation/index.html#margin-and-gap">Computation: margin and gap</a>
    */
   double get_gap();
}
