package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class WrenchSensorDefinition extends SensorDefinition
{
   public WrenchSensorDefinition()
   {
   }

   public WrenchSensorDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public WrenchSensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      super(name, transformToJoint);
   }

   public WrenchSensorDefinition(WrenchSensorDefinition other)
   {
      super(other);
   }

   @Override
   public SensorDefinition copy()
   {
      return new WrenchSensorDefinition(this);
   }
}
