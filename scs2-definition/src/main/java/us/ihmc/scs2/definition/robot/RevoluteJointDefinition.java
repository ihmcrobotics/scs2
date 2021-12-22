package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class RevoluteJointDefinition extends OneDoFJointDefinition
{
   public RevoluteJointDefinition()
   {
   }

   public RevoluteJointDefinition(String name)
   {
      super(name);
   }

   public RevoluteJointDefinition(String name, Tuple3DReadOnly offsetFromParent, Vector3DReadOnly axis)
   {
      super(name, offsetFromParent, axis);
   }

   public RevoluteJointDefinition(RevoluteJointDefinition other)
   {
      super(other);
   }

   @Override
   public RevoluteJointBasics toJoint(RigidBodyBasics predecessor)
   {
      RevoluteJoint joint = new RevoluteJoint(getName(), predecessor, getTransformToParent(), getAxis());
      joint.setJointLimits(getPositionLowerLimit(), getPositionUpperLimit());
      joint.setVelocityLimits(getVelocityLowerLimit(), getVelocityUpperLimit());
      joint.setEffortLimits(getEffortLowerLimit(), getEffortUpperLimit());
      return joint;
   }

   @Override
   public RevoluteJointDefinition copy()
   {
      return new RevoluteJointDefinition(this);
   }
}
