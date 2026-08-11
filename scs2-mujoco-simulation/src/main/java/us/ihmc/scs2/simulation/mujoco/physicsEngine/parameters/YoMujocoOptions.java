package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The single home of every live-tunable value: the full runtime-writable {@code mjOption} struct
 * plus SCS2's own {@code subSteps}, in a {@code MujocoOptions} child registry; editing any variable
 * here changes the physics engine. The MJCF deliberately emits none of them. Defaults match
 * {@code mj_defaultOption} except the noted SCS2 deviations; {@code timestep} and {@code gravity}
 * are engine-owned and absent.
 * <p>
 * Names are verbatim {@code mjOption} field names, so each one is directly searchable: googling
 * e.g. "mujoco solimp" or "mujoco impratio" lands on the reference. The relevant pages are
 * <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters">Modeling &gt;
 * Solver parameters</a> (solref/solimp semantics; the Contact override section there is exactly
 * the o_* / enableOverride mechanism) and the
 * <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#option">XML reference,
 * option element</a>.
 * Array-valued fields get a component suffix — a "shim name" that is not itself an API symbol,
 * taken from the docs' parameter tuples (solref = (timeconst, dampratio), solimp = (dmin, dmax,
 * width, midpoint, power)); each description states the exact index it maps to. The {@code o_} prefix is MuJoCo's "override"
 * marker: these mirror {@code mjOption}'s contact-override group (active when enableOverride),
 * as opposed to the per-geom defaults of the same name in MujocoSimulationParameters. Naming
 * convention module-wide: MuJoCo-derived names are snake_case verbatim; SCS2-owned names are
 * camelCase; members are grouped SCS2 first, then MuJoCo.
 * </p>
 * <p>
 * Edits from any thread only trip a dirty flag (a plain boolean so GUI buffer scrubbing cannot
 * replay it); the engine consumes it via {@link #pollUpdateRequest()} and pushes the values into
 * {@code mjModel.opt} on the physics thread — including once at compile, so values set before the
 * first tick apply from step one.
 * </p>
 * <p>
 * Rule of thumb for where a value belongs: if the {@code mjOption} struct accepts a change between
 * steps (global solver and contact-override settings), it is an option and goes here, live-tunable
 * from the GUI; if MuJoCo would need a model recompile to honor it (per-geom/per-joint attributes,
 * collision structure, allocation sizes), it is a parameter and belongs in
 * {@link MujocoSimulationParametersReadOnly}.
 * </p>
 */
public class YoMujocoOptions
{
   private final YoRegistry registry = new YoRegistry("MujocoOptions");
   private boolean updateOptionsRequested = false;
   private boolean subStepsChangeRequested = false;

   // ---------- SCS2-owned ----------

   /** Not an {@code mjOption} field — SCS2's own stepping knob, placed here because it is live. */
   public final YoInteger subSteps = var("subSteps",
         "SCS2-owned, not an mjOption field: mj_step calls per SCS2 tick; MuJoCo timestep = session dt / subSteps", 1);
   public final YoBoolean enableOverride = var("enableOverride",
         "mjENBL_OVERRIDE: o_* values replace margin/solref/solimp/friction on EVERY contact", false);
   public final YoBoolean enableEnergy = var("enableEnergy",
         "mjENBL_ENERGY: compute energy into the MujocoStatistics energy variables", false);

   // ---------- MuJoCo-owned (mjOption, in struct order) ----------

   public final YoDouble impratio = var("impratio",
         "Frictional-to-normal constraint impedance ratio; raise (with cone = ELLIPTIC) to fight slip", 1.0);
   public final YoDouble tolerance = var("tolerance",
         "Main solver termination tolerance; smaller = more accurate, more iterations", 1.0e-8);
   public final YoDouble ls_tolerance = var("ls_tolerance",
         "CG/Newton line-search termination tolerance; rarely tuned", 0.01);
   public final YoDouble noslip_tolerance = var("noslip_tolerance",
         "Noslip post-pass termination tolerance (active when noslip_iterations > 0)", 1.0e-6);
   public final YoDouble ccd_tolerance = var("ccd_tolerance",
         "Convex-mesh narrowphase (GJK/EPA) termination tolerance", 1.0e-6);
   public final YoDouble o_margin = var("o_margin",
         "Override contact detection margin [m]", 0.0);
   public final YoDouble o_solref_timeconst = var("o_solref_timeconst",
         "Override solref[0], contact settling time constant [s]; smaller = harder contact, keep >= 2*timestep", 0.02);
   public final YoDouble o_solref_dampratio = var("o_solref_dampratio",
         "Override solref[1], contact damping ratio; 1 = critically damped, < 1 bouncy", 1.0);
   public final YoDouble o_solimp_dmin = var("o_solimp_dmin",
         "Override solimp[0] (solver impedance dmin): hardness at first touch, 0-1; lower = softer", 0.9);
   public final YoDouble o_solimp_dmax = var("o_solimp_dmax",
         "Override solimp[1] (dmax): hardness once penetrated past width, 0-1", 0.95);
   public final YoDouble o_solimp_width = var("o_solimp_width",
         "Override solimp[2] (width) [m]: penetration depth over which hardness ramps dmin to dmax", 0.001);
   public final YoDouble o_solimp_midpoint = var("o_solimp_midpoint",
         "Override solimp[3] (midpoint): ramp inflection point, 0-1 fraction of width", 0.5);
   public final YoDouble o_solimp_power = var("o_solimp_power",
         "Override solimp[4] (power): ramp sharpness, >= 1 (1 = linear)", 2.0);
   public final YoDouble o_friction_slide = var("o_friction_slide",
         "Override sliding friction, both tangential directions (o_friction[0..1])", 1.0);
   public final YoDouble o_friction_spin = var("o_friction_spin",
         "Override torsional (spin) friction about the contact normal (o_friction[2])", 0.005);
   public final YoDouble o_friction_roll = var("o_friction_roll",
         "Override rolling friction (o_friction[3..4])", 1.0e-4);
   public final YoEnum<MujocoIntegrator> integrator = var("integrator",
         "Integrator (mjtIntegrator); SCS2 default IMPLICITFAST handles high joint damping, MuJoCo default EULER", MujocoIntegrator.IMPLICITFAST);
   public final YoEnum<MujocoCone> cone = var("cone",
         "Friction cone model (mjtCone); ELLIPTIC + impratio > 1 fights slip", MujocoCone.PYRAMIDAL);
   public final YoEnum<MujocoJacobian> jacobian = var("jacobian",
         "Constraint Jacobian storage (mjtJacobian); performance only", MujocoJacobian.AUTO);
   public final YoEnum<MujocoSolver> solver = var("solver",
         "Constraint solver (mjtSolver); NEWTON converges fastest", MujocoSolver.NEWTON);
   public final YoInteger iterations = var("iterations",
         "Max main solver iterations per step (SCS2 default 25, MuJoCo 100)", 25);
   public final YoInteger ls_iterations = var("ls_iterations",
         "Max line-search iterations per solver iteration", 50);
   public final YoInteger noslip_iterations = var("noslip_iterations",
         "Non-physical noslip post-pass iterations suppressing residual contact drift; 0 disables (SCS2 default 5, MuJoCo 0)", 5);
   public final YoInteger ccd_iterations = var("ccd_iterations",
         "Max convex-mesh narrowphase (GJK/EPA) iterations", 50);

   public YoMujocoOptions(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      // Attached after the defaults above, so construction leaves the dirty flags untouched.
      // subSteps gets its own flag: it is not an mjOption field, so its edits must not trigger a
      // native option push, and the engine must only pick it up through the poll (buffer scrubs
      // restore values without firing listeners, so polled changes are deliberate edits only).
      registry.getVariables().forEach(variable -> variable.addListener(v -> updateOptionsRequested = true));
      subSteps.removeListeners();
      subSteps.addListener(v -> subStepsChangeRequested = true);
   }

   /** Consume-and-clear the subSteps dirty flag; physics thread only. */
   public boolean pollSubStepsRequest()
   {
      boolean requested = subStepsChangeRequested;
      subStepsChangeRequested = false;
      return requested;
   }

   /**
    * Consume-and-clear the dirty flag. Physics thread only; apply the values to the native model
    * when this returns true — cleared before applying so a concurrent edit is not swallowed.
    */
   public boolean pollUpdateRequest()
   {
      boolean requested = updateOptionsRequested;
      updateOptionsRequested = false;
      return requested;
   }

   private YoDouble var(String name, String description, double initialValue)
   {
      YoDouble variable = new YoDouble(name, description, registry);
      variable.set(initialValue);
      return variable;
   }

   private YoInteger var(String name, String description, int initialValue)
   {
      YoInteger variable = new YoInteger(name, description, registry);
      variable.set(initialValue);
      return variable;
   }

   private YoBoolean var(String name, String description, boolean initialValue)
   {
      YoBoolean variable = new YoBoolean(name, description, registry);
      variable.set(initialValue);
      return variable;
   }

   private <E extends Enum<E>> YoEnum<E> var(String name, String description, E initialValue)
   {
      YoEnum<E> variable = new YoEnum<>(name, description, registry, initialValue.getDeclaringClass(), false);
      variable.set(initialValue);
      return variable;
   }
}
