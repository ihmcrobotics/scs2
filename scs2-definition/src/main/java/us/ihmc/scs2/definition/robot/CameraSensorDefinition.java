package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import jakarta.xml.bind.annotation.XmlElement;
import java.util.Objects;

public class CameraSensorDefinition extends SensorDefinition
{
   private boolean enable;

   private double fieldOfView;
   private double clipNear;
   private double clipFar;

   private int imageWidth;
   private int imageHeight;

   /**
    * Selects the axis in the sensor frame towards which the camera is looking at.
    */
   private Vector3D depthAxis = new Vector3D(Axis3D.X);
   /**
    * Selects the axis in the sensor frame that represents the up direction.
    */
   private Vector3D upAxis = new Vector3D(Axis3D.Z);

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
      depthAxis.set(other.depthAxis);
      upAxis.set(other.upAxis);
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

   public Vector3D getDepthAxis()
   {
      return depthAxis;
   }

   @XmlElement
   public void setDepthAxis(Vector3D depthAxis)
   {
      this.depthAxis = depthAxis;
   }

   public void setDepthAxis(Vector3DReadOnly depthAxis)
   {
      this.depthAxis = new Vector3D(depthAxis);
   }

   public Vector3D getUpAxis()
   {
      return upAxis;
   }

   @XmlElement
   public void setUpAxis(Vector3D upAxis)
   {
      this.upAxis = upAxis;
   }

   public void setUpAxis(Vector3DReadOnly upAxis)
   {
      this.upAxis = new Vector3D(upAxis);
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
      bits = EuclidHashCodeTools.addToHashCode(bits, depthAxis);
      bits = EuclidHashCodeTools.addToHashCode(bits, upAxis);
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
      if (!EuclidCoreTools.equals(fieldOfView, other.fieldOfView))
         return false;
      if (!EuclidCoreTools.equals(clipNear, other.clipNear))
         return false;
      if (!EuclidCoreTools.equals(clipFar, other.clipFar))
         return false;
      if (imageWidth != other.imageWidth)
         return false;
      if (imageHeight != other.imageHeight)
         return false;
      if (!Objects.equals(depthAxis, other.depthAxis))
         return false;
      if (!Objects.equals(upAxis, other.upAxis))
         return false;
      return true;
   }
}
