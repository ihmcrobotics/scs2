package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The single home of every runtime-tunable {@code mjOption} value, in a {@code MujocoOptions} child
 * registry with MuJoCo's documented names; the MJCF deliberately emits none of them. Defaults match
 * {@code mj_defaultOption} except the three noted SCS2 deviations; {@code timestep} and
 * {@code gravity} are engine-owned and absent. Edits from any thread only trip a dirty flag (a
 * plain boolean so GUI buffer scrubbing cannot replay it); the engine consumes it via
 * {@link #pollUpdateRequest()} and pushes the values into {@code mjModel.opt} on the physics
 * thread — including once at compile, so values set before the first tick apply from step one.
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

   public final YoDouble impratio = var("impratio", "Frictional-to-normal constraint impedance ratio; > 1 stiffens friction (anti-slip, elliptic cones)", 1.0);
   public final YoDouble tolerance = var("tolerance", "Main solver convergence tolerance", 1.0e-8);
   public final YoDouble ls_tolerance = var("ls_tolerance", "CG/Newton line-search early-termination tolerance", 0.01);
   public final YoDouble noslip_tolerance = var("noslip_tolerance", "Convergence tolerance of the noslip post-pass", 1.0e-6);
   public final YoDouble ccd_tolerance = var("ccd_tolerance", "Convex collision (GJK/EPA) early-termination tolerance", 1.0e-6);
   public final YoInteger iterations = var("iterations", "Maximum main solver iterations (SCS2 default 25; MuJoCo default 100)", 25);
   public final YoInteger ls_iterations = var("ls_iterations", "Maximum line-search iterations per solver iteration", 50);
   public final YoInteger noslip_iterations = var("noslip_iterations", "Noslip post-processing iterations; 0 disables (SCS2 default 5; MuJoCo default 0)", 5);
   public final YoInteger ccd_iterations = var("ccd_iterations", "Maximum iterations of the convex collision routine", 50);
   public final YoEnum<MujocoSolver> solver = var("solver", "Constraint solver algorithm (mjtSolver)", MujocoSolver.NEWTON);
   public final YoEnum<MujocoCone> cone = var("cone", "Friction cone model (mjtCone)", MujocoCone.PYRAMIDAL);
   public final YoEnum<MujocoJacobian> jacobian = var("jacobian", "Constraint Jacobian representation (mjtJacobian, performance only)", MujocoJacobian.AUTO);
   public final YoEnum<MujocoIntegrator> integrator = var("integrator", "Numerical integrator (mjtIntegrator; SCS2 default IMPLICITFAST, MuJoCo default EULER)", MujocoIntegrator.IMPLICITFAST);
   public final YoBoolean enableOverride = var("enableOverride", "mjENBL_OVERRIDE: o_* values override margin/solref/solimp/friction for EVERY contact", false);
   public final YoBoolean enableEnergy = var("enableEnergy", "mjENBL_ENERGY: compute potential/kinetic energy into mjData.energy", false);
   public final YoDouble o_margin = var("o_margin", "Override contact detection margin [m] (active when enableOverride)", 0.0);
   public final YoDouble o_solrefTimeconst = var("o_solrefTimeconst", "o_solref[0]: contact settling time constant [s]; keep >= 2*timestep", 0.02);
   public final YoDouble o_solrefDampratio = var("o_solrefDampratio", "o_solref[1]: contact damping ratio; 1 = critically damped (no bounce)", 1.0);
   public final YoDouble o_solimpDmin = var("o_solimpDmin", "o_solimp[0]: impedance at first touch (softness at the surface)", 0.9);
   public final YoDouble o_solimpDmax = var("o_solimpDmax", "o_solimp[1]: impedance once fully penetrated past width", 0.95);
   public final YoDouble o_solimpWidth = var("o_solimpWidth", "o_solimp[2]: softening-zone thickness [m] before full hardness", 0.001);
   public final YoDouble o_solimpMidpoint = var("o_solimpMidpoint", "o_solimp[3]: impedance sigmoid inflection point (curve shape)", 0.5);
   public final YoDouble o_solimpPower = var("o_solimpPower", "o_solimp[4]: impedance sigmoid sharpness, >= 1 (curve shape)", 2.0);
   public final YoDouble o_frictionSlide = var("o_frictionSlide", "o_friction[0..1]: sliding friction coefficient", 1.0);
   public final YoDouble o_frictionSpin = var("o_frictionSpin", "o_friction[2]: torsional (spin) friction coefficient", 0.005);
   public final YoDouble o_frictionRoll = var("o_frictionRoll", "o_friction[3..4]: rolling friction coefficient", 1.0e-4);

   public YoMujocoOptions(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      // Attached after the defaults above, so construction leaves the dirty flag untouched.
      registry.getVariables().forEach(variable -> variable.addListener(v -> updateOptionsRequested = true));
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
