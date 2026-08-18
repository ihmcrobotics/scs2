package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Read-write view of the MuJoCo simulation parameters, implemented by the
 * {@link MujocoSimulationParameters} POJO. Members are grouped SCS2-owned (camelCase) first, then
 * MuJoCo-owned (snake_case).
 */
public interface MujocoSimulationParametersBasics extends MujocoSimulationParametersReadOnly
{
   default void set(MujocoSimulationParametersReadOnly other)
   {
      setSubSteps(other.getSubSteps());
      setTimestep(other.getTimestep());
      setFilterParentCollisions(other.getFilterParentCollisions());
      setPerContactDiagnosticsCapacity(other.getPerContactDiagnosticsCapacity());

      set_solref_timeconst(other.get_solref_timeconst());
      set_solref_dampratio(other.get_solref_dampratio());
      set_solimp_dmin(other.get_solimp_dmin());
      set_solimp_dmax(other.get_solimp_dmax());
      set_solimp_width(other.get_solimp_width());
      set_solimp_midpoint(other.get_solimp_midpoint());
      set_solimp_power(other.get_solimp_power());
      set_friction_slide(other.get_friction_slide());
      set_friction_spin(other.get_friction_spin());
      set_friction_roll(other.get_friction_roll());
      set_condim(other.get_condim());
      set_margin(other.get_margin());
      set_gap(other.get_gap());
      set_armature(other.get_armature());
      other.get_armature_overrides().forEach(this::set_armature);
   }

   // ---------- SCS2-owned ----------

   void setSubSteps(int subSteps);

   /**
    * @deprecated Never read: the effective MuJoCo timestep is the session dt divided by
    *             {@link #getSubSteps()}, set every tick in {@code MujocoPhysicsEngine.simulate}.
    */
   @Deprecated
   void setTimestep(double timestep);

   void setFilterParentCollisions(boolean filterParentCollisions);

   void setPerContactDiagnosticsCapacity(int perContactDiagnosticsCapacity);

   // ---------- MuJoCo-owned ----------

   void set_solref_timeconst(double solref_timeconst);

   void set_solref_dampratio(double solref_dampratio);

   void set_solimp_dmin(double solimp_dmin);

   void set_solimp_dmax(double solimp_dmax);

   void set_solimp_width(double solimp_width);

   void set_solimp_midpoint(double solimp_midpoint);

   void set_solimp_power(double solimp_power);

   void set_friction_slide(double friction_slide);

   void set_friction_spin(double friction_spin);

   void set_friction_roll(double friction_roll);

   void set_condim(int condim);

   void set_margin(double margin);

   void set_gap(double gap);

   void set_armature(double armature);

   /** Sets a per-joint {@link #set_armature(double)} override; see {@link MujocoSimulationParametersReadOnly#get_armature_overrides()}. */
   void set_armature(String jointName, double armature);
}
