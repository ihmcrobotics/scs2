package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjSolverStat_;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjWarningStat_;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Per-step MuJoCo solver/constraint diagnostics, read after every {@code mj_step} with the names
 * the MuJoCo documentation uses. Per-phase timers ({@code mjData.timer}) are absent: they need the
 * {@code mjcb_time} callback, which the JavaCPP binding does not map yet.
 */
public class MujocoPhysicsEngineStatistics
{
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

   public MujocoPhysicsEngineStatistics(YoRegistry registry)
   {
      ncon = new YoInteger("ncon", "Number of detected contacts this step", registry);
      nefc = new YoInteger("nefc", "Total number of constraint rows this step", registry);
      ne = new YoInteger("ne", "Number of equality constraint rows (contact rows = nefc - ne - nf - nl)", registry);
      nf = new YoInteger("nf", "Number of friction-loss constraint rows", registry);
      nl = new YoInteger("nl", "Number of limit constraint rows", registry);
      nisland = new YoInteger("nisland", "Number of detected constraint islands (solver stats below cover island 0 only)", registry);
      solver_niter = new YoInteger("solver_niter", "Solver iterations used this step (island 0)", registry);
      solverImprovement = new YoDouble("solverImprovement", "Last-iteration cost reduction, scaled by 1/trace(M(qpos0)) (island 0)", registry);
      solverGradient = new YoDouble("solverGradient", "Last-iteration gradient norm, primal solvers only (island 0)", registry);
      solverNactive = new YoInteger("solverNactive", "Number of active constraints at the last iteration (island 0)", registry);
      solverNchange = new YoInteger("solverNchange", "Constraint state changes at the last iteration (island 0)", registry);
      warnInertia = new YoInteger("warnInertia", "Cumulative mjWARN_INERTIA count: (near) singular inertia matrix", registry);
      warnContactFull = new YoInteger("warnContactFull", "Cumulative mjWARN_CONTACTFULL count: too many contacts", registry);
      warnCnstrFull = new YoInteger("warnCnstrFull", "Cumulative mjWARN_CNSTRFULL count: too many constraints", registry);
      warnBadQpos = new YoInteger("warnBadQpos", "Cumulative mjWARN_BADQPOS count: bad number in qpos", registry);
      warnBadQvel = new YoInteger("warnBadQvel", "Cumulative mjWARN_BADQVEL count: bad number in qvel", registry);
      warnBadQacc = new YoInteger("warnBadQacc", "Cumulative mjWARN_BADQACC count: bad number in qacc — earliest sign of a bad contact-parameter set", registry);
      warnBadCtrl = new YoInteger("warnBadCtrl", "Cumulative mjWARN_BADCTRL count: bad number in ctrl", registry);
      energyPotential = new YoDouble("energyPotential", "Potential energy (NaN unless enableEnergy is set in the MujocoOptions registry)", registry);
      energyKinetic = new YoDouble("energyKinetic", "Kinetic energy (NaN unless enableEnergy is set in the MujocoOptions registry)", registry);
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
