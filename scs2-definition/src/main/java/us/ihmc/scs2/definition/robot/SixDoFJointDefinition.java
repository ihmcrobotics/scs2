package us.ihmc.scs2.definition.robot;

import java.util.Objects;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public class SixDoFJointDefinition extends JointDefinition
{
   private SixDoFJointState initialJointState;

   // For compatibility with SCS1
   @Deprecated
   private String variableName;

   public SixDoFJointDefinition()
   {
   }

   public SixDoFJointDefinition(String name)
   {
      super(name);
   }

   public SixDoFJointDefinition(String name, Tuple3DReadOnly offsetFromParent)
   {
      super(name, offsetFromParent);
   }

   public SixDoFJointDefinition(SixDoFJointDefinition other)
   {
      super(other);
      initialJointState = other.initialJointState == null ? null : other.initialJointState.copy();
   }

   // For compatibility with SCS1
   @Deprecated
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
   }

   // For compatibility with SCS1
   @Deprecated
   public String getVariableName()
   {
      return variableName;
   }

   public void setInitialJointState(SixDoFJointState initialJointState)
   {
      this.initialJointState = initialJointState;
   }

   @Override
   public void setInitialJointState(JointStateReadOnly initialJointState)
   {
      if (initialJointState instanceof SixDoFJointState)
         setInitialJointState((SixDoFJointState) initialJointState);
      else if (this.initialJointState == null)
         this.initialJointState = new SixDoFJointState(initialJointState);
      else
         this.initialJointState.set(initialJointState);
   }

   @Override
   public SixDoFJointState getInitialJointState()
   {
      return initialJointState;
   }

   @Override
   public SixDoFJointBasics toJoint(RigidBodyBasics predecessor)
   {
      return new SixDoFJoint(getName(), predecessor, getTransformToParent());
   }

   @Override
   public SixDoFJointDefinition copy()
   {
      return new SixDoFJointDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, initialJointState);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      SixDoFJointDefinition other = (SixDoFJointDefinition) object;

      if (!Objects.equals(initialJointState, other.initialJointState))
         return false;
      if (!Objects.equals(variableName, other.variableName))
         return false;

      return true;
   }
}
