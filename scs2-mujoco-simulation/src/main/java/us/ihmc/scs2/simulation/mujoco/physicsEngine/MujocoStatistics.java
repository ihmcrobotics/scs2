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
 * one exists; derived names state their source in the description. Per-phase timers
 * ({@code mjData.timer}) are absent: they need the {@code mjcb_time} callback, unmapped in the
 * binding. Per-body contact force totals live in each robot's own registry, not here.
 */
public class MujocoStatistics
{
   private final YoRegistry registry = new YoRegistry("MujocoStatistics");

   // Engine-written timing/state gauges; values are pushed by MujocoPhysicsEngine each tick.
   public final YoDouble mujocoRealtimeRate;
   public final YoDouble mujocoSimulateTimeMs;
   public final YoLong mujocoTick;
   public final YoTimer stepTimer;
   public final YoBoolean hasBeenCompiled;

   private final YoInteger ncon;
   private final YoInteger nefc;
   private final YoInteger ne;
   private final YoInteger nf;
   private final YoInteger nl;
   private final YoInteger nisland;
   private final YoInteger solver_niter;
   private final YoDouble solverImprovement;
   private final YoDouble solverGradient;
   private final YoInteger solverNactive;
   private final YoInteger solverNchange;
   private final YoInteger warnInertia;
   private final YoInteger warnContactFull;
   private final YoInteger warnCnstrFull;
   private final YoInteger warnBadQpos;
   private final YoInteger warnBadQvel;
   private final YoInteger warnBadQacc;
   private final YoInteger warnBadCtrl;
   private final YoDouble energyPotential;
   private final YoDouble energyKinetic;

   private mjData data;
   // Cached JavaCPP wrappers over mjData's stat arrays; the mjData arena is stable for the life of
   // the model, so binding once keeps the per-tick loop allocation-free.
   private mjWarningStat_ warningStats;
   private mjSolverStat_ solverStats;

   public MujocoStatistics(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      mujocoRealtimeRate = new YoDouble("MujocoRealtimeRate", "Achieved sim-time / wall-time rate, windowed", registry);
      mujocoSimulateTimeMs = new YoDouble("MujocoSimulateTime[ms]", "Wall time between simulate() calls", registry);
      mujocoTick = new YoLong("MujocoTick", "Engine tick counter", registry);
      stepTimer = new YoTimer("mujocoStep", TimeUnit.MILLISECONDS, registry);
      hasBeenCompiled = new YoBoolean("mujocoHasBeenCompiled", "True once the MJCF world has been compiled; display only, re-asserted every step", registry);
      ncon = new YoInteger("ncon", "Contacts detected this step (mjData.ncon)", registry);
      nefc = new YoInteger("nefc", "Total constraint rows this step (mjData.nefc)", registry);
      ne = new YoInteger("ne", "Equality constraint rows; contact rows = nefc - ne - nf - nl", registry);
      nf = new YoInteger("nf", "Friction-loss constraint rows", registry);
      nl = new YoInteger("nl", "Joint/tendon limit constraint rows", registry);
      nisland = new YoInteger("nisland", "Constraint islands detected; solver stats below cover island 0 only", registry);
      solver_niter = new YoInteger("solver_niter", "Solver iterations used this step (island 0); pinned at the iterations cap = not converging", registry);
      solverImprovement = new YoDouble("solverImprovement", "mjSolverStat.improvement at the last iteration: cost reduction, near zero when converged", registry);
      solverGradient = new YoDouble("solverGradient", "mjSolverStat.gradient at the last iteration: gradient norm, small when converged (primal solvers)", registry);
      solverNactive = new YoInteger("solverNactive", "mjSolverStat.nactive at the last iteration: active constraints", registry);
      solverNchange = new YoInteger("solverNchange", "mjSolverStat.nchange at the last iteration: constraint state changes", registry);
      warnInertia = new YoInteger("warnInertia", "Cumulative mjWARN_INERTIA count: (near) singular inertia matrix; any increase mid-run is trouble", registry);
      warnContactFull = new YoInteger("warnContactFull", "Cumulative mjWARN_CONTACTFULL count: too many contacts", registry);
      warnCnstrFull = new YoInteger("warnCnstrFull", "Cumulative mjWARN_CNSTRFULL count: too many constraints", registry);
      warnBadQpos = new YoInteger("warnBadQpos", "Cumulative mjWARN_BADQPOS count: bad number in qpos", registry);
      warnBadQvel = new YoInteger("warnBadQvel", "Cumulative mjWARN_BADQVEL count: bad number in qvel", registry);
      warnBadQacc = new YoInteger("warnBadQacc", "Cumulative mjWARN_BADQACC count: bad number in qacc, the earliest sign of a bad contact-parameter set", registry);
      warnBadCtrl = new YoInteger("warnBadCtrl", "Cumulative mjWARN_BADCTRL count: bad number in ctrl", registry);
      energyPotential = new YoDouble("energyPotential", "mjData.energy[0]: potential energy; NaN unless MujocoOptions enableEnergy", registry);
      energyKinetic = new YoDouble("energyKinetic", "mjData.energy[1]: kinetic energy; NaN unless MujocoOptions enableEnergy", registry);
      energyPotential.set(Double.NaN);
      energyKinetic.set(Double.NaN);
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
      nefc.set(data.nefc());
      ne.set(data.ne());
      nf.set(data.nf());
      nl.set(data.nl());
      nisland.set(data.nisland());

      int niter = data.solver_niter(0);
      solver_niter.set(niter);
      if (niter > 0)
      {
         // mjData.solver is laid out island-major (mjNISLAND arrays of mjNSOLVER entries); island
         // 0's iteration i is simply index i.
         int lastIteration = Math.min(niter, Mujoco.mjNSOLVER) - 1;
         solverStats.position(lastIteration);
         solverImprovement.set(solverStats.improvement());
         solverGradient.set(solverStats.gradient());
         solverNactive.set(solverStats.nactive());
         solverNchange.set(solverStats.nchange());
      }
      else
      {
         solverImprovement.set(0.0);
         solverGradient.set(0.0);
         solverNactive.set(0);
         solverNchange.set(0);
      }

      warnInertia.set(warningStats.position(Mujoco.mjWARN_INERTIA).number());
      warnContactFull.set(warningStats.position(Mujoco.mjWARN_CONTACTFULL).number());
      warnCnstrFull.set(warningStats.position(Mujoco.mjWARN_CNSTRFULL).number());
      warnBadQpos.set(warningStats.position(Mujoco.mjWARN_BADQPOS).number());
      warnBadQvel.set(warningStats.position(Mujoco.mjWARN_BADQVEL).number());
      warnBadQacc.set(warningStats.position(Mujoco.mjWARN_BADQACC).number());
      warnBadCtrl.set(warningStats.position(Mujoco.mjWARN_BADCTRL).number());
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
         energyPotential.set(data.energy(0));
         energyKinetic.set(data.energy(1));
      }
      else
      {
         energyPotential.set(Double.NaN);
         energyKinetic.set(Double.NaN);
      }
   }
}
