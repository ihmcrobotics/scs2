package us.ihmc.scs2.simulation.robot.multiBodySystem;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoRevoluteTwinsJoint;
import us.ihmc.scs2.definition.robot.RevoluteTwinsJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimRevoluteTwinsJoint extends YoRevoluteTwinsJoint implements SimOneDoFJointBasics, RevoluteTwinsJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final TwistReadOnly jointDeltaTwist;
   private final YoDouble deltaQd;
   private final YoBoolean isPinned;
   private final YoDouble damping;

   public SimRevoluteTwinsJoint(RevoluteTwinsJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(),
           predecessor,
           definition.getJointA().getName(),
           definition.getJointB().getName(),
           definition.getBodyAB().getName(),
           definition.getTransformAToPredecessor(),
           definition.getTransformBToA(),
           definition.getBodyAB().getMomentOfInertia(),
           definition.getBodyAB().getMass(),
           definition.getBodyAB().getInertiaPose(),
           definition.getActuatedJointIndex(),
           definition.getConstraintRatio(),
           definition.getConstraintOffset(),
           definition.getAxis());

      setJointLimits(definition.getPositionLowerLimit(), definition.getPositionUpperLimit());
      setVelocityLimits(definition.getVelocityLowerLimit(), definition.getVelocityUpperLimit());
      setEffortLimits(definition.getEffortLowerLimit(), definition.getEffortUpperLimit());
      setDamping(definition.getDamping());

      var jointADefinition = definition.getJointA();
      getJointA().setJointLimits(jointADefinition.getPositionLowerLimit(), jointADefinition.getPositionUpperLimit());
      getJointA().setVelocityLimits(jointADefinition.getVelocityLowerLimit(), jointADefinition.getVelocityUpperLimit());
      getJointA().setEffortLimits(jointADefinition.getEffortLowerLimit(), jointADefinition.getEffortUpperLimit());

      var jointBDefinition = definition.getJointB();
      getJointB().setJointLimits(jointBDefinition.getPositionLowerLimit(), jointBDefinition.getPositionUpperLimit());
      getJointB().setVelocityLimits(jointBDefinition.getVelocityLowerLimit(), jointBDefinition.getVelocityUpperLimit());
      getJointB().setEffortLimits(jointBDefinition.getEffortLowerLimit(), jointBDefinition.getEffortUpperLimit());
   }

   public SimRevoluteTwinsJoint(String name,
                                SimRigidBodyBasics predecessor,
                                String jointNameA,
                                String jointNameB,
                                String bodyNameAB,
                                RigidBodyTransformReadOnly transformAToPredecessor,
                                RigidBodyTransformReadOnly transformBToA,
                                Matrix3DReadOnly bodyInertiaAB,
                                double bodyMassAB,
                                RigidBodyTransformReadOnly bodyInertiaPoseAB,
                                int actuatedJointIndex,
                                double constraintRatio,
                                double constraintOffset,
                                Vector3DReadOnly jointAxis)
   {
      super(name,
            predecessor,
            jointNameA,
            jointNameB,
            bodyNameAB,
            transformAToPredecessor,
            transformBToA,
            bodyInertiaAB,
            bodyMassAB,
            bodyInertiaPoseAB,
            actuatedJointIndex,
            constraintRatio,
            constraintOffset,
            jointAxis,
            predecessor.getRegistry());

      this.registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);
      deltaQd = new YoDouble("qd_delta_" + getName(), registry);
      jointDeltaTwist = MecanoFactories.newTwistReadOnly(this::getDeltaQd, getUnitJointTwist());
      isPinned = new YoBoolean("is_" + getName() + "_pinned", registry);
      damping = new YoDouble("damping_" + getName(), registry);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointAuxiliaryData getAuxiliaryData()
   {
      return auxiliaryData;
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      if (successor instanceof SimRigidBodyBasics)
         super.setSuccessor(successor);
      else
         throw new IllegalArgumentException("Can only set a " + SimRigidBodyBasics.class.getSimpleName() + " as successor of a " + getClass().getSimpleName());
   }

   @Override
   public SimRigidBodyBasics getPredecessor()
   {
      return (SimRigidBodyBasics) super.getPredecessor();
   }

   @Override
   public SimRigidBodyBasics getSuccessor()
   {
      return (SimRigidBodyBasics) super.getSuccessor();
   }

   @Override
   public void resetState()
   {
      SimOneDoFJointBasics.super.resetState();
      updateFrame();
   }

   @Override
   public void setQ(double q)
   {
      if (!Double.isFinite(q))
         throw new IllegalStateException("Invalid joint configuration: " + q);

      super.setQ(q);
   }

   @Override
   public double computeActuatedJointQ(double q)
   {
      if (!Double.isFinite(q))
         throw new IllegalStateException("Invalid joint configuration: " + q);

      double actuatedJointQ = super.computeActuatedJointQ(q);

      if (!Double.isFinite(actuatedJointQ))
         throw new IllegalStateException("Invalid joint configuration: " + actuatedJointQ);

      return actuatedJointQ;
   }

   @Override
   public double computeActuatedJointQdd(double qdd)
   {
      if (!Double.isFinite(qdd))
         throw new IllegalStateException("Invalid joint acceleration: " + qdd);

      double actuatedJointQdd = super.computeActuatedJointQdd(qdd);

      if (!Double.isFinite(actuatedJointQdd))
         throw new IllegalStateException("Invalid joint acceleration: " + actuatedJointQdd);

      return actuatedJointQdd;
   }

   @Override
   public double getDamping()
   {
      return damping.getValue();
   }

   @Override
   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   @Override
   public double getDeltaQd()
   {
      return deltaQd.getValue();
   }

   @Override
   public void setDeltaQd(double deltaQd)
   {
      this.deltaQd.set(deltaQd);
   }

   @Override
   public void setJointAngularDeltaVelocity(Vector3DReadOnly jointAngularDeltaVelocity)
   {
      setDeltaQd(getJointAxis().dot(jointAngularDeltaVelocity));
   }

   @Override
   public void setJointLinearDeltaVelocity(Vector3DReadOnly jointLinearDeltaVelocity)
   {
   }

   @Override
   public TwistReadOnly getJointDeltaTwist()
   {
      return jointDeltaTwist;
   }

   @Override
   public void setPinned(boolean isPinned)
   {
      this.isPinned.set(isPinned);
   }

   @Override
   public boolean isPinned()
   {
      return isPinned.getValue();
   }
}
