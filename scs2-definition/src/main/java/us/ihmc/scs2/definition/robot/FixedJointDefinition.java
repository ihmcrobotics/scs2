package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.FixedJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public class FixedJointDefinition extends JointDefinition
{
   public FixedJointDefinition()
   {
   }

   public FixedJointDefinition(String name)
   {
      super(name);
   }

   public FixedJointDefinition(String name, Tuple3DReadOnly offsetFromParent)
   {
      super(name, offsetFromParent);
   }

   public FixedJointDefinition(FixedJointDefinition other)
   {
      super(other);
   }

   @Override
   public void setInitialJointState(JointStateReadOnly initialJointState)
   {
   }

   @Override
   public JointStateBasics getInitialJointState()
   {
      return null;
   }

   @Override
   public FixedJointBasics toJoint(RigidBodyBasics predecessor)
   {
      return new FixedJoint(getName(), predecessor, getTransformToParent());
   }

   @Override
   public FixedJointDefinition copy()
   {
      return new FixedJointDefinition(this);
   }
}
