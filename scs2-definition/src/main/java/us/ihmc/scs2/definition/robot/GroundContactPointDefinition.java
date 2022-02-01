package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class GroundContactPointDefinition extends ExternalWrenchPointDefinition
{
   private int groupIdentifier = 0;

   public GroundContactPointDefinition()
   {
      super();
   }

   public GroundContactPointDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public GroundContactPointDefinition(String name, Tuple3DReadOnly offsetFromJoint, int groupIdentifier)
   {
      super(name, offsetFromJoint);
      this.groupIdentifier = groupIdentifier;
   }

   public GroundContactPointDefinition(String name, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, transformToParent);
   }

   public GroundContactPointDefinition(String name, RigidBodyTransformReadOnly transformToParent, int groupIdentifier)
   {
      super(name, transformToParent);
      this.groupIdentifier = groupIdentifier;
   }

   public GroundContactPointDefinition(GroundContactPointDefinition other)
   {
      super(other);
      groupIdentifier = other.groupIdentifier;
   }

   public void setGroupIdentifier(int groupIdentifier)
   {
      this.groupIdentifier = groupIdentifier;
   }

   public int getGroupIdentifier()
   {
      return groupIdentifier;
   }

   @Override
   public GroundContactPointDefinition copy()
   {
      return new GroundContactPointDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, groupIdentifier);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      GroundContactPointDefinition other = (GroundContactPointDefinition) object;

      if (groupIdentifier != other.groupIdentifier)
         return false;

      return true;
   }
}
