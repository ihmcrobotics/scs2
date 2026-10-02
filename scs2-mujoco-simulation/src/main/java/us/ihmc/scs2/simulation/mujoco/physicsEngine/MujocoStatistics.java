package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.log.LogTools;
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
   public final YoDouble solver_fwdinv_qfrc, solver_fwdinv_efc;
   /**
    * Impact severity, independent of how many contact slots are configured. The touchdown transient
    * is what the state estimator has to survive, and these are the three numbers that describe it:
    * how deep anything is penetrating, how hard the hardest contact is pushing, and how fast that
    * force is changing. A rigid solver delivers an impulse as one enormous tick of force, so the
    * rate is often more diagnostic than the peak.
    */
   public final YoDouble maxPenetrationDepth, maxContactNormalForce, maxContactNormalForceRate;
   public final YoInteger maxPenetrationGeomA, maxPenetrationGeomB;

   /**
    * MuJoCo reports its own trouble through {@code mjData.warning[*]} and, separately, by appending to
    * MUJOCO_LOG.TXT in the working directory -- which during a test run is invisible. These counters
    * are already mirrored as YoVariables below, but nothing watches them, so a NaN in qacc or a
    * constraint-buffer overflow passes silently. This logs the first increment of each, loudly.
    * Costs one int comparison per counter per tick.
    */
   private static final boolean LOG_MUJOCO_WARNINGS = !"0".equals(System.getenv("SCS2_MUJOCO_LOG_WARNINGS"));

   private final int[] loggedWarningCounts = new int[8];
   private double previousMaxContactNormalForce = 0.0;
   private final YoDouble energy_kinetic;

   private mjModel model;
   private mjData data;
   private final org.bytedeco.javacpp.DoublePointer contactForceScratch = new org.bytedeco.javacpp.DoublePointer(6);
   // Cached JavaCPP wrappers over mjData's stat arrays; the mjData arena is stable for the life of
   // the model, so binding once keeps the per-tick loop allocation-free.
   private mjWarningStat_ warningStats;
   private mjSolverStat_ solverStats;

   public MujocoStatistics(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      realtimeRate = new YoDouble("realtimeRate", "Achieved sim-time / wall-time rate, windowed", registry);
      maxPenetrationDepth = new YoDouble("maxPenetrationDepth",
                                         "Deepest contact penetration this tick [m], >= 0; 0 when nothing is touching",
                                         registry);
      maxPenetrationGeomA = new YoInteger("maxPenetrationGeomA",
                                          "mjModel geom id of the first geom in the deepest-penetrating contact, -1 when there is none",
                                          registry);
      maxPenetrationGeomB = new YoInteger("maxPenetrationGeomB",
                                          "mjModel geom id of the second geom in the deepest-penetrating contact, -1 when there is none",
                                          registry);
      maxContactNormalForce = new YoDouble("maxContactNormalForce", "Largest single contact normal force this tick [N]", registry);
      maxContactNormalForceRate = new YoDouble("maxContactNormalForceRate",
                                               "Change in maxContactNormalForce since the previous tick [N/tick]",
                                               registry);
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
      solver_improvement = new YoDouble("solver_improvement", "mjSolverStat_.improvement at the last iteration: cost reduction, near zero when converged", registry);
      solver_gradient = new YoDouble("solver_gradient", "mjSolverStat_.gradient at the last iteration: gradient norm, small when converged (primal solvers)", registry);
      solver_nactive = new YoInteger("solver_nactive", "mjSolverStat_.nactive at the last iteration: active constraints", registry);
      solver_nchange = new YoInteger("solver_nchange", "mjSolverStat_.nchange at the last iteration: constraint state changes", registry);
      warning_inertia = new YoInteger("warning_inertia", "Cumulative mjWARN_INERTIA count: (near) singular inertia matrix; any increase mid-run is trouble", registry);
      warning_contactfull = new YoInteger("warning_contactfull", "Cumulative mjWARN_CONTACTFULL count: too many contacts", registry);
      warning_cnstrfull = new YoInteger("warning_cnstrfull", "Cumulative mjWARN_CNSTRFULL count: too many constraints", registry);
      warning_badqpos = new YoInteger("warning_badqpos", "Cumulative mjWARN_BADQPOS count: bad number in qpos", registry);
      warning_badqvel = new YoInteger("warning_badqvel", "Cumulative mjWARN_BADQVEL count: bad number in qvel", registry);
      warning_badqacc = new YoInteger("warning_badqacc", "Cumulative mjWARN_BADQACC count: bad number in qacc, the earliest sign of a bad contact-parameter set", registry);
      warning_badctrl = new YoInteger("warning_badctrl", "Cumulative mjWARN_BADCTRL count: bad number in ctrl", registry);
      energy_potential = new YoDouble("energy_potential", "mjData.energy[0]: potential energy; NaN unless MujocoOptions enableEnergy", registry);
      energy_kinetic = new YoDouble("energy_kinetic", "mjData.energy[1]: kinetic energy; NaN unless MujocoOptions enableEnergy", registry);
      solver_fwdinv_qfrc = new YoDouble("solver_fwdinv_qfrc",
                                        "mjData.solver_fwdinv[0]: forward-vs-inverse dynamics discrepancy in generalized force; "
                                        + "a direct measure of how well the constraint solver converged. NaN unless MujocoOptions enableFwdinv",
                                        registry);
      solver_fwdinv_efc = new YoDouble("solver_fwdinv_efc",
                                       "mjData.solver_fwdinv[1]: the same comparison in constraint space. "
                                       + "NaN unless MujocoOptions enableFwdinv",
                                       registry);
      energy_potential.set(Double.NaN);
      energy_kinetic.set(Double.NaN);
      solver_fwdinv_qfrc.set(Double.NaN);
      solver_fwdinv_efc.set(Double.NaN);
   }

   /** Caches native pointers; call once, right after the model has compiled. */
   public void bind(mjModel model, mjData data)
   {
      this.model = model;
      this.data = data;
      warningStats = new mjWarningStat_(data.warning(0));
      solverStats = new mjSolverStat_(data.solver(0));
   }

   /** Scans mjData.contact for the tick's worst penetration and force; cheap, ncon is small. */
   private void updateContactSeverity()
   {
      if (data == null)
      {
         maxPenetrationDepth.setToNaN();
         maxPenetrationGeomA.set(-1);
         maxPenetrationGeomB.set(-1);
         maxContactNormalForce.setToNaN();
         maxContactNormalForceRate.setToNaN();
         return;
      }

      int ncon = data.ncon();
      double deepest = 0.0;
      double strongest = 0.0;
      int deepestGeomA = -1;
      int deepestGeomB = -1;
      if (ncon > 0)
      {
         Mujoco.mjContact contact = data.contact();
         for (int i = 0; i < ncon; i++)
         {
            contact.position(i);
            double depth = -contact.dist();
            if (depth > deepest)
            {
               deepest = depth;
               // Attribute the maximum: without the geom pair this is a bare number over every
               // contact in the world, and a deep penetration between two unrelated geoms reads
               // exactly like a foot sinking through the floor.
               deepestGeomA = contact.geom1();
               deepestGeomB = contact.geom2();
            }
            Mujoco.mj_contactForce(model, data, i, contactForceScratch);
            strongest = Math.max(strongest, contactForceScratch.get(0));
         }
      }
      maxPenetrationDepth.set(deepest);
      maxPenetrationGeomA.set(deepestGeomA);
      maxPenetrationGeomB.set(deepestGeomB);
      maxContactNormalForce.set(strongest);
      maxContactNormalForceRate.set(strongest - previousMaxContactNormalForce);
      previousMaxContactNormalForce = strongest;
   }

   /** Reads the diagnostics of the step that just completed; call after stepping, on the physics thread. */
   public void update()
   {
      updateContactSeverity();
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

      if (LOG_MUJOCO_WARNINGS)
         logNewMujocoWarnings();
   }



   /** Logs the first increment of each MuJoCo warning counter. See {@link #LOG_MUJOCO_WARNINGS}. */
   private void logNewMujocoWarnings()
   {
      logIfNew(0, Mujoco.mjWARN_INERTIA, "INERTIA: (near) singular inertia matrix");
      logIfNew(1, Mujoco.mjWARN_CONTACTFULL, "CONTACTFULL: too many contacts, some were dropped");
      logIfNew(2, Mujoco.mjWARN_CNSTRFULL, "CNSTRFULL: constraint buffer full, constraints were dropped");
      logIfNew(3, Mujoco.mjWARN_BADQPOS, "BADQPOS: bad number in qpos");
      logIfNew(4, Mujoco.mjWARN_BADQVEL, "BADQVEL: bad number in qvel");
      logIfNew(5, Mujoco.mjWARN_BADQACC, "BADQACC: bad number in qacc -- earliest sign of a bad contact-parameter set");
      logIfNew(6, Mujoco.mjWARN_BADCTRL, "BADCTRL: bad number in ctrl");
   }

   private void logIfNew(int slot, int warningBit, String description)
   {
      int count = warningStats.position(warningBit).number();
      if (count > loggedWarningCounts[slot])
      {
         loggedWarningCounts[slot] = count;
         LogTools.warn(String.format("MuJoCo mjWARN_%s (count now %d, tick %d)", description, count, tick.getValue()));
      }
   }

   /**
    * Reads {@code mjData.energy} when the energy flag is enabled, NaN otherwise -- MuJoCo leaves
    * zeros in the array when {@code mjENBL_ENERGY} is off, which would read as a plausible value.
    */
   /**
    * Reads the forward-vs-inverse dynamics comparison MuJoCo writes when {@code mjENBL_FWDINV} is
    * set: {@code ||qfrc_constraint_forward - qfrc_constraint_inverse||} over all nv degrees of
    * freedom, and the same comparison over the nefc constraint rows.
    * <p>
    * <b>Do not read this as a convergence measure when contacts have friction.</b> It was added to
    * answer "is the constraint solver converging?" and it cannot. {@code mj_inverse} reconstructs
    * {@code efc_force} analytically from the constraint acceleration, and for a STICKING contact the
    * tangential constraint acceleration is zero, so it reconstructs roughly zero friction no matter
    * what static friction force the forward solve actually chose. Static friction is
    * inequality-constrained: it is not a function of acceleration, so the comparison is structurally
    * invalid for frictional contact at rest, and the number it returns is the size of the stiction
    * being carried.
    * <p>
    * Measured on a statically resting Zulu (no controller, 9 contacts, residual joint motion
    * 1.2e-4 rad/s): <b>42.8 with {@code condim == 4} and 5.1e-12 with {@code condim == 1}</b>,
    * identical model, identical contact set -- a ratio of 8.4e12. Per-row decomposition shows the
    * residual sitting on the elliptic contacts' friction rows, where forward returns tens of newtons
    * and inverse returns ~0; only the normal rows differ by anything resembling a solver margin
    * (around 5%). A symmetric box resting flat reads 1e-12 for the same reason -- it has no
    * tangential demand -- which is what makes naive probe comparisons so misleading here.
    * <p>
    * It remains meaningful with {@code condim == 1}, in free flight, and as a relative signal at a
    * fixed friction configuration. The norm also mixes units: newtons on a free joint's three
    * translational DoF, newton-metres on the rest.
    */
   public void updateSolverDiagnostics(boolean fwdinvEnabled)
   {
      if (data == null)
         return;

      if (fwdinvEnabled)
      {
         solver_fwdinv_qfrc.set(data.solver_fwdinv(0));
         solver_fwdinv_efc.set(data.solver_fwdinv(1));
      }
      else
      {
         // Deliberately NaN rather than left stale: with the flag off MuJoCo does not touch the
         // array, and a frozen plausible-looking number reads as a converged solver.
         solver_fwdinv_qfrc.set(Double.NaN);
         solver_fwdinv_efc.set(Double.NaN);
      }
   }

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
