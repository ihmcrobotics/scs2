package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Compile-time MuJoCo simulation seeds; see {@link MujocoSimulationParametersReadOnly}. This class
 * is the single home of the default values. Members are grouped SCS2-owned (camelCase) first, then
 * MuJoCo-owned (snake_case).
 */
public class MujocoSimulationParameters implements MujocoSimulationParametersBasics
{
   private int subSteps = 1;
   private double timestep = 0.0;
   private boolean filterParentCollisions = true;
   private int perContactDiagnosticsCapacity = 16;

   private double solref_timeconst = 0.02;
   private double solref_dampratio = 1.0;
   private double solimp_dmin = 0.9;
   private double solimp_dmax = 0.99;
   private double solimp_width = 0.0007;
   private double solimp_midpoint = 0.5;
   private double solimp_power = 2.0;
   private double friction_slide = 1.0;
   private double friction_spin = 0.05;
   private double friction_roll = 0.01;
   private int condim = 4;
   private double margin = 0.0;
   private double gap = 0.0;
   private double armature = 0.0;
   private final Map<String, Double> armatureOverrides = new HashMap<>();

   public static MujocoSimulationParameters defaultMujocoSimulationParameters()
   {
      return new MujocoSimulationParameters();
   }

   // ---------- SCS2-owned ----------

   @Override
   public int getSubSteps()
   {
      return subSteps;
   }

   @Override
   public void setSubSteps(int subSteps)
   {
      this.subSteps = subSteps;
   }

   @Deprecated
   @Override
   public double getTimestep()
   {
      return timestep;
   }

   @Deprecated
   @Override
   public void setTimestep(double timestep)
   {
      this.timestep = timestep;
   }

   @Override
   public boolean getFilterParentCollisions()
   {
      return filterParentCollisions;
   }

   @Override
   public void setFilterParentCollisions(boolean filterParentCollisions)
   {
      this.filterParentCollisions = filterParentCollisions;
   }

   @Override
   public int getPerContactDiagnosticsCapacity()
   {
      return perContactDiagnosticsCapacity;
   }

   @Override
   public void setPerContactDiagnosticsCapacity(int perContactDiagnosticsCapacity)
   {
      this.perContactDiagnosticsCapacity = perContactDiagnosticsCapacity;
   }

   // ---------- MuJoCo-owned ----------

   @Override
   public double get_solref_timeconst()
   {
      return solref_timeconst;
   }

   @Override
   public void set_solref_timeconst(double solref_timeconst)
   {
      this.solref_timeconst = solref_timeconst;
   }

   @Override
   public double get_solref_dampratio()
   {
      return solref_dampratio;
   }

   @Override
   public void set_solref_dampratio(double solref_dampratio)
   {
      this.solref_dampratio = solref_dampratio;
   }

   @Override
   public double get_solimp_dmin()
   {
      return solimp_dmin;
   }

   @Override
   public void set_solimp_dmin(double solimp_dmin)
   {
      this.solimp_dmin = solimp_dmin;
   }

   @Override
   public double get_solimp_dmax()
   {
      return solimp_dmax;
   }

   @Override
   public void set_solimp_dmax(double solimp_dmax)
   {
      this.solimp_dmax = solimp_dmax;
   }

   @Override
   public double get_solimp_width()
   {
      return solimp_width;
   }

   @Override
   public void set_solimp_width(double solimp_width)
   {
      this.solimp_width = solimp_width;
   }

   @Override
   public double get_solimp_midpoint()
   {
      return solimp_midpoint;
   }

   @Override
   public void set_solimp_midpoint(double solimp_midpoint)
   {
      this.solimp_midpoint = solimp_midpoint;
   }

   @Override
   public double get_solimp_power()
   {
      return solimp_power;
   }

   @Override
   public void set_solimp_power(double solimp_power)
   {
      this.solimp_power = solimp_power;
   }

   @Override
   public double get_friction_slide()
   {
      return friction_slide;
   }

   @Override
   public void set_friction_slide(double friction_slide)
   {
      this.friction_slide = friction_slide;
   }

   @Override
   public double get_friction_spin()
   {
      return friction_spin;
   }

   @Override
   public void set_friction_spin(double friction_spin)
   {
      this.friction_spin = friction_spin;
   }

   @Override
   public double get_friction_roll()
   {
      return friction_roll;
   }

   @Override
   public void set_friction_roll(double friction_roll)
   {
      this.friction_roll = friction_roll;
   }

   @Override
   public int get_condim()
   {
      return condim;
   }

   @Override
   public void set_condim(int condim)
   {
      this.condim = condim;
   }

   @Override
   public double get_margin()
   {
      return margin;
   }

   @Override
   public void set_margin(double margin)
   {
      this.margin = margin;
   }

   @Override
   public double get_gap()
   {
      return gap;
   }

   @Override
   public void set_gap(double gap)
   {
      this.gap = gap;
   }

   @Override
   public double get_armature()
   {
      return armature;
   }

   @Override
   public void set_armature(double armature)
   {
      this.armature = armature;
   }

   @Override
   public Map<String, Double> get_armature_overrides()
   {
      return Collections.unmodifiableMap(armatureOverrides);
   }

   @Override
   public void set_armature(String jointName, double armature)
   {
      armatureOverrides.put(jointName, armature);
   }
}
