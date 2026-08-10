package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Read-write view of the MuJoCo simulation parameters, implemented by the
 * {@link MujocoSimulationParameters} POJO.
 */
public interface MujocoSimulationParametersBasics extends MujocoSimulationParametersReadOnly
{
   default void set(MujocoSimulationParametersReadOnly other)
   {
      setSubSteps(other.getSubSteps());
      setContactSolrefTimeconst(other.getContactSolrefTimeconst());
      setContactSolrefDampRatio(other.getContactSolrefDampRatio());
      setContactSolimpDmin(other.getContactSolimpDmin());
      setContactSolimpDmax(other.getContactSolimpDmax());
      setJointArmature(other.getJointArmature());
      setFrictionSlide(other.getFrictionSlide());
      setTimestep(other.getTimestep());
      setFilterParentCollisions(other.getFilterParentCollisions());
      setFrictionTorsional(other.getFrictionTorsional());
      setFrictionRolling(other.getFrictionRolling());
      setContactSolimpWidth(other.getContactSolimpWidth());
      setContactSolimpMidpoint(other.getContactSolimpMidpoint());
      setContactSolimpPower(other.getContactSolimpPower());
      setCondim(other.getCondim());
      setContactMargin(other.getContactMargin());
      setContactGap(other.getContactGap());
      setPerContactDiagnosticsCapacity(other.getPerContactDiagnosticsCapacity());
   }

   void setSubSteps(int subSteps);

   void setContactSolrefTimeconst(double contactSolrefTimeconst);

   void setContactSolrefDampRatio(double contactSolrefDampRatio);

   void setContactSolimpDmin(double contactSolimpDmin);

   void setContactSolimpDmax(double contactSolimpDmax);

   void setJointArmature(double jointArmature);

   void setFrictionSlide(double frictionSlide);

   /**
    * @deprecated Never read: the effective MuJoCo timestep is the session dt divided by
    *             {@link #getSubSteps()}, set every tick in {@code MujocoPhysicsEngine.simulate}.
    */
   @Deprecated
   void setTimestep(double timestep);

   void setFilterParentCollisions(boolean filterParentCollisions);

   void setFrictionTorsional(double frictionTorsional);

   void setFrictionRolling(double frictionRolling);

   void setContactSolimpWidth(double contactSolimpWidth);

   void setContactSolimpMidpoint(double contactSolimpMidpoint);

   void setContactSolimpPower(double contactSolimpPower);

   void setCondim(int condim);

   void setContactMargin(double contactMargin);

   void setContactGap(double contactGap);

   void setPerContactDiagnosticsCapacity(int perContactDiagnosticsCapacity);
}
