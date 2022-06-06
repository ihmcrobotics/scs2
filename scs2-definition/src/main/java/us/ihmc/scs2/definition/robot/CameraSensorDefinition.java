package us.ihmc.scs2.definition.robot;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class CameraSensorDefinition extends SensorDefinition
{
   private boolean enable;

   private double fieldOfView;
   private double clipNear;
   private double clipFar;

   private int imageWidth;
   private int imageHeight;

   public CameraSensorDefinition()
   {
   }

   public CameraSensorDefinition(String name)
   {
      super(name);
   }

   public CameraSensorDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public CameraSensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      super(name, transformToJoint);
   }

   public CameraSensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint, double fieldOfView, double clipNear, double clipFar)
   {
      super(name, transformToJoint);

      setFieldOfView(fieldOfView);

      setClipNear(clipNear);
      setClipFar(clipFar);
   }

   public CameraSensorDefinition(CameraSensorDefinition other)
   {
      super(other);

      enable = other.enable;
      fieldOfView = other.fieldOfView;
      clipNear = other.clipNear;
      clipFar = other.clipFar;
      imageWidth = other.imageWidth;
      imageHeight = other.imageHeight;
   }

   public boolean getEnable()
   {
      return enable;
   }

   @XmlElement
   public void setEnable(boolean enable)
   {
      this.enable = enable;
   }

   public double getFieldOfView()
   {
      return fieldOfView;
   }

   @XmlElement
   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   public double getClipNear()
   {
      return clipNear;
   }

   @XmlElement
   public void setClipNear(double clipNear)
   {
      this.clipNear = clipNear;
   }

   public double getClipFar()
   {
      return clipFar;
   }

   @XmlElement
   public void setClipFar(double clipFar)
   {
      this.clipFar = clipFar;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   @XmlElement
   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   @XmlElement
   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   @Override
   public CameraSensorDefinition copy()
   {
      return new CameraSensorDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, enable);
      bits = EuclidHashCodeTools.addToHashCode(bits, fieldOfView);
      bits = EuclidHashCodeTools.addToHashCode(bits, clipNear);
      bits = EuclidHashCodeTools.addToHashCode(bits, clipFar);
      bits = EuclidHashCodeTools.addToHashCode(bits, imageWidth);
      bits = EuclidHashCodeTools.addToHashCode(bits, imageHeight);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;
      CameraSensorDefinition other = (CameraSensorDefinition) object;
      if (enable != other.enable)
         return false;
      if (Double.doubleToLongBits(fieldOfView) != Double.doubleToLongBits(other.fieldOfView))
         return false;
      if (Double.doubleToLongBits(clipNear) != Double.doubleToLongBits(other.clipNear))
         return false;
      if (Double.doubleToLongBits(clipFar) != Double.doubleToLongBits(other.clipFar))
         return false;
      if (imageWidth != other.imageWidth)
         return false;
      if (imageHeight != other.imageHeight)
         return false;
      return true;
   }
}
