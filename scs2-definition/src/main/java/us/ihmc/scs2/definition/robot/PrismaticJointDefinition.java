package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class PrismaticJointDefinition extends OneDoFJointDefinition
{
   public PrismaticJointDefinition()
   {
   }

   public PrismaticJointDefinition(String name)
   {
      super(name);
   }

   public PrismaticJointDefinition(String name, Tuple3DReadOnly offsetFromParent, Vector3DReadOnly axis)
   {
      super(name, offsetFromParent, axis);
   }

   public PrismaticJointDefinition(PrismaticJointDefinition other)
   {
      super(other);
   }

   @Override
   public PrismaticJointBasics toJoint(RigidBodyBasics predecessor)
   {
      PrismaticJoint joint = new PrismaticJoint(getName(), predecessor, getTransformToParent(), getAxis());
      joint.setJointLimits(getPositionLowerLimit(), getPositionUpperLimit());
      joint.setVelocityLimits(getVelocityLowerLimit(), getVelocityUpperLimit());
      joint.setEffortLimits(getEffortLowerLimit(), getEffortUpperLimit());
      return joint;
   }

   @Override
   public PrismaticJointDefinition copy()
   {
      return new PrismaticJointDefinition(this);
   }
}
