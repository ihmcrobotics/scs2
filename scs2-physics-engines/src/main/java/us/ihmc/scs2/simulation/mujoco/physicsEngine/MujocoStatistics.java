package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjSolverStat_;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjWarningStat_;
import java.util.concurrent.TimeUnit;

import us.ihmc.scs2.session.YoTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Read-only per-step diagnostics in a {@code MujocoStatistics} child registry; editing these does
 * nothing (the engine overwrites them every step). Names are verbatim {@code mjData} fields where
 * one exists; derived names state their source in the description. MuJoCo-derived names are
 * snake_case verbatim; SCS2-owned names (timing, hasBeenCompiled) are camelCase. Per-phase timers
 * ({@code mjData.timer}) are absent: they need the {@code mjcb_time} callback, unmapped in the
 * binding. Per-body contact force totals live in each robot's own registry, not here.
 */
public class MujocoStatistics
{
   private final YoRegistry registry = new YoRegistry("MujocoStatistics");

   // ---------- SCS2-owned (engine-written gauges) ----------
   public final YoDouble realtimeRate;
   public final YoDouble simulateTime;
   public final YoLong tick;
   public final YoTimer stepTimer;
   public final YoBoolean hasBeenCompiled;

   // ---------- MuJoCo-owned (mjData, in struct order) ----------

   private final YoInteger ncon;
   private final YoInteger ne;
   private final YoInteger nf;
   private final YoInteger nl;
   private final YoInteger nefc;
   private final YoInteger nisland;
   private final YoInteger solver_niter;
   private final YoDouble solver_improvement;
   private final YoDouble solver_gradient;
   private final YoInteger solver_nactive;
   private final YoInteger solver_nchange;
   private final YoInteger warning_inertia;
   private final YoInteger warning_contactfull;
   private final YoInteger warning_cnstrfull;
   private final YoInteger warning_badqpos;
   private final YoInteger warning_badqvel;
   private final YoInteger warning_badqacc;
   private final YoInteger warning_badctrl;
   private final YoDouble energy_potential;
   private final YoDouble energy_kinetic;

   private mjData data;
   // Cached JavaCPP wrappers over mjData's stat arrays; the mjData arena is stable for the life of
   // the model, so binding once keeps the per-tick loop allocation-free.
   private mjWarningStat_ warningStats;
   private mjSolverStat_ solverStats;

   public MujocoStatistics(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      realtimeRate = new YoDouble("realtimeRate", "Achieved sim-time / wall-time rate, windowed", registry);
      simulateTime = new YoDouble("simulateTime[ms]", "Wall time between simulate() calls", registry);
      tick = new YoLong("tick", "Engine tick counter", registry);
      stepTimer = new YoTimer("step", TimeUnit.MILLISECONDS, registry);
      hasBeenCompiled = new YoBoolean("hasBeenCompiled", "True once the MJCF world has been compiled; display only, re-asserted every step", registry);

      ncon = new YoInteger("ncon", "Contacts detected this step (mjData.ncon)", registry);
      ne = new YoInteger("ne", "Equality constraint rows; contact rows = nefc - ne - nf - nl", registry);
      nf = new YoInteger("nf", "Friction-loss constraint rows", registry);
      nl = new YoInteger("nl", "Joint/tendon limit constraint rows", registry);
      nefc = new YoInteger("nefc", "Total constraint rows this step (mjData.nefc)", registry);
      nisland = new YoInteger("nisland", "Constraint islands detected; solver stats below cover island 0 only", registry);
      solver_niter = new YoInteger("solver_niter", "Solver iterations used this step (island 0); pinned at the iterations cap = not converging", registry);
      solver_improvement = new YoDouble("solver_improvement", "mjSolverStat.improvement at the last iteration: cost reduction, near zero when converged", registry);
      solver_gradient = new YoDouble("solver_gradient", "mjSolverStat.gradient at the last iteration: gradient norm, small when converged (primal solvers)", registry);
      solver_nactive = new YoInteger("solver_nactive", "mjSolverStat.nactive at the last iteration: active constraints", registry);
      solver_nchange = new YoInteger("solver_nchange", "mjSolverStat.nchange at the last iteration: constraint state changes", registry);
      warning_inertia = new YoInteger("warning_inertia", "Cumulative mjWARN_INERTIA count: (near) singular inertia matrix; any increase mid-run is trouble", registry);
      warning_contactfull = new YoInteger("warning_contactfull", "Cumulative mjWARN_CONTACTFULL count: too many contacts", registry);
      warning_cnstrfull = new YoInteger("warning_cnstrfull", "Cumulative mjWARN_CNSTRFULL count: too many constraints", registry);
      warning_badqpos = new YoInteger("warning_badqpos", "Cumulative mjWARN_BADQPOS count: bad number in qpos", registry);
      warning_badqvel = new YoInteger("warning_badqvel", "Cumulative mjWARN_BADQVEL count: bad number in qvel", registry);
      warning_badqacc = new YoInteger("warning_badqacc", "Cumulative mjWARN_BADQACC count: bad number in qacc, the earliest sign of a bad contact-parameter set", registry);
      warning_badctrl = new YoInteger("warning_badctrl", "Cumulative mjWARN_BADCTRL count: bad number in ctrl", registry);
      energy_potential = new YoDouble("energy_potential", "mjData.energy[0]: potential energy; NaN unless MujocoOptions enableEnergy", registry);
      energy_kinetic = new YoDouble("energy_kinetic", "mjData.energy[1]: kinetic energy; NaN unless MujocoOptions enableEnergy", registry);
      energy_potential.set(Double.NaN);
      energy_kinetic.set(Double.NaN);
   }

   /** Caches native pointers; call once, right after the model has compiled. */
   public void bind(mjModel model, mjData data)
   {
      this.data = data;
      warningStats = new mjWarningStat_(data.warning(0));
      solverStats = new mjSolverStat_(data.solver(0));
   }

   /** Reads the diagnostics of the step that just completed; call after stepping, on the physics thread. */
   public void update()
   {
      if (data == null)
         return;

      hasBeenCompiled.set(true); // Re-asserted so GUI edits/buffer scrubs cannot make this gauge lie.

      ncon.set(data.ncon());
      ne.set(data.ne());
      nf.set(data.nf());
      nl.set(data.nl());
      nefc.set(data.nefc());
      nisland.set(data.nisland());

      int niter = data.solver_niter(0);
      solver_niter.set(niter);
      if (niter > 0)
      {
         // mjData.solver is laid out island-major (mjNISLAND arrays of mjNSOLVER entries); island
         // 0's iteration i is simply index i.
         int lastIteration = Math.min(niter, Mujoco.mjNSOLVER) - 1;
         solverStats.position(lastIteration);
         solver_improvement.set(solverStats.improvement());
         solver_gradient.set(solverStats.gradient());
         solver_nactive.set(solverStats.nactive());
         solver_nchange.set(solverStats.nchange());
      }
      else
      {
         solver_improvement.set(0.0);
         solver_gradient.set(0.0);
         solver_nactive.set(0);
         solver_nchange.set(0);
      }

      warning_inertia.set(warningStats.position(Mujoco.mjWARN_INERTIA).number());
      warning_contactfull.set(warningStats.position(Mujoco.mjWARN_CONTACTFULL).number());
      warning_cnstrfull.set(warningStats.position(Mujoco.mjWARN_CNSTRFULL).number());
      warning_badqpos.set(warningStats.position(Mujoco.mjWARN_BADQPOS).number());
      warning_badqvel.set(warningStats.position(Mujoco.mjWARN_BADQVEL).number());
      warning_badqacc.set(warningStats.position(Mujoco.mjWARN_BADQACC).number());
      warning_badctrl.set(warningStats.position(Mujoco.mjWARN_BADCTRL).number());
   }

   /**
    * Reads {@code mjData.energy} when the energy flag is enabled, NaN otherwise — MuJoCo leaves
    * zeros in the array when {@code mjENBL_ENERGY} is off, which would read as a plausible value.
    */
   public void updateEnergy(boolean energyEnabled)
   {
      if (data == null)
         return;

      if (energyEnabled)
      {
         energy_potential.set(data.energy(0));
         energy_kinetic.set(data.energy(1));
      }
      else
      {
         energy_potential.set(Double.NaN);
         energy_kinetic.set(Double.NaN);
      }
   }
}
