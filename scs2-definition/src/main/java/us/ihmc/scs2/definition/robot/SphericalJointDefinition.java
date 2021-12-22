package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.scs2.definition.state.SphericalJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public class SphericalJointDefinition extends JointDefinition
{
   private SphericalJointState initialJointState;

   public SphericalJointDefinition()
   {
   }

   public SphericalJointDefinition(String name)
   {
      super(name);
   }

   public SphericalJointDefinition(String name, Tuple3DReadOnly offsetFromParent)
   {
      super(name, offsetFromParent);
   }

   public SphericalJointDefinition(SphericalJointDefinition other)
   {
      super(other);
      initialJointState = other.initialJointState == null ? null : other.initialJointState.copy();
   }

   public void setInitialJointState(SphericalJointState initialJointState)
   {
      this.initialJointState = initialJointState;
   }

   @Override
   public void setInitialJointState(JointStateReadOnly initialJointState)
   {
      if (initialJointState instanceof SphericalJointState)
         setInitialJointState((SphericalJointState) initialJointState);
      else if (this.initialJointState == null)
         this.initialJointState = new SphericalJointState(initialJointState);
      else
         this.initialJointState.set(initialJointState);
   }

   @Override
   public SphericalJointState getInitialJointState()
   {
      return initialJointState;
   }

   @Override
   public SphericalJointBasics toJoint(RigidBodyBasics predecessor)
   {
      return new SphericalJoint(getName(), predecessor, getTransformToParent());
   }

   @Override
   public SphericalJointDefinition copy()
   {
      return new SphericalJointDefinition(this);
   }
}
