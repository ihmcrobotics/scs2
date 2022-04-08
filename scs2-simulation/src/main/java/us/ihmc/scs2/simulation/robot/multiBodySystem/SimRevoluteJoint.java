package us.ihmc.scs2.simulation.robot.multiBodySystem;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoRevoluteJoint;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimRevoluteJoint extends YoRevoluteJoint implements SimOneDoFJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final YoDouble deltaQd;
   private final YoBoolean isPinned;

   private final TwistReadOnly jointDeltaTwist;

   public SimRevoluteJoint(RevoluteJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(), predecessor, definition.getTransformToParent(), definition.getAxis());
      setJointLimits(definition.getPositionLowerLimit(), definition.getPositionUpperLimit());
      setVelocityLimits(definition.getVelocityLowerLimit(), definition.getVelocityUpperLimit());
      setEffortLimits(definition.getEffortLowerLimit(), definition.getEffortUpperLimit());
   }

   public SimRevoluteJoint(String name, SimRigidBodyBasics predecessor, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, (RigidBodyTransformReadOnly) null, jointAxis);
   }

   public SimRevoluteJoint(String name, SimRigidBodyBasics predecessor, Tuple3DReadOnly jointOffset, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset), jointAxis);
   }

   public SimRevoluteJoint(String name, SimRigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, transformToParent, jointAxis, predecessor.getRegistry());
      registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);
      deltaQd = new YoDouble("qd_delta_" + getName(), registry);
      jointDeltaTwist = MecanoFactories.newTwistReadOnly(this::getDeltaQd, getUnitJointTwist());
      isPinned = new YoBoolean("is_" + getName() + "_pinned", registry);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointAuxiliaryData getAuxialiryData()
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
   public void setQ(double q)
   {
      if (!Double.isFinite(q))
         throw new IllegalStateException("Invalid joint configuration: " + q);
      super.setQ(q);
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
