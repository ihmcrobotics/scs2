package us.ihmc.scs2.definition.robot;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;

public class SensorDefinition implements Transformable
{
   private String name;
   private YawPitchRollTransformDefinition transformToJoint = new YawPitchRollTransformDefinition();

   // Period in milliseconds
   private int updatePeriod;

   public SensorDefinition()
   {
   }

   public SensorDefinition(String name)
   {
      this.name = name;
   }

   public SensorDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      this.name = name;
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public SensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      this.name = name;
      this.transformToJoint.set(transformToJoint);
   }

   public SensorDefinition(SensorDefinition other)
   {
      name = other.name;
      transformToJoint.set(other.transformToJoint);
      updatePeriod = other.updatePeriod;
   }

   public String getName()
   {
      return name;
   }

   @XmlElement
   public void setName(String name)
   {
      this.name = name;
   }

   public YawPitchRollTransformDefinition getTransformToJoint()
   {
      return transformToJoint;
   }

   public void setOffsetFromJoint(Tuple3DReadOnly offsetFromJoint)
   {
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   @XmlElement
   public void setTransformToJoint(YawPitchRollTransformDefinition transformToJoint)
   {
      this.transformToJoint = transformToJoint;
   }

   public void setTransformToJoint(RigidBodyTransformReadOnly transformToJoint)
   {
      this.transformToJoint.set(transformToJoint);
   }

   public int getUpdatePeriod()
   {
      return updatePeriod;
   }

   /**
    * Sets the interval in milliseconds between updates for this sensor.
    * 
    * @param updatePeriod the update period in milliseconds.
    */
   @XmlElement
   public void setUpdatePeriod(int updatePeriod)
   {
      this.updatePeriod = updatePeriod;
   }

   public SensorDefinition copy()
   {
      return new SensorDefinition(this);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(transformToJoint);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(transformToJoint);
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " - " + name;
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, transformToJoint);
      bits = EuclidHashCodeTools.addToHashCode(bits, updatePeriod);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      SensorDefinition other = (SensorDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(transformToJoint, other.transformToJoint))
         return false;
      if (updatePeriod != other.updatePeriod)
         return false;

      return true;
   }
}
