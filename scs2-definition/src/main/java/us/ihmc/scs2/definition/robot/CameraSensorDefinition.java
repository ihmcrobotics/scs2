package us.ihmc.scs2.definition.robot;

import javax.xml.bind.annotation.XmlElement;

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
}
