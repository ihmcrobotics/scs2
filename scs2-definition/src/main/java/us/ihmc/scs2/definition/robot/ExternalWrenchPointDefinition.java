package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ExternalWrenchPointDefinition extends KinematicPointDefinition
{
   public ExternalWrenchPointDefinition()
   {
      super();
   }

   public ExternalWrenchPointDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public ExternalWrenchPointDefinition(String name, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, transformToParent);
   }

   public ExternalWrenchPointDefinition(ExternalWrenchPointDefinition other)
   {
      super(other);
   }

   @Override
   public ExternalWrenchPointDefinition copy()
   {
      return new ExternalWrenchPointDefinition(this);
   }
}
