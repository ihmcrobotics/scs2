package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicCapsule3D")
public class YoGraphicCapsule3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition center;
   private YoTuple3DDefinition axis;

   private String length;
   private String radius;

   public YoGraphicCapsule3DDefinition()
   {
      registerTuple3DField("center", this::getCenter, this::setCenter);
      registerTuple3DField("axis", this::getAxis, this::setAxis);
      registerStringField("length", this::getLength, this::setLength);
      registerStringField("radius", this::getRadius, this::setRadius);
   }

   @XmlElement
   public void setCenter(YoTuple3DDefinition center)
   {
      this.center = center;
   }

   @XmlElement
   public void setAxis(YoTuple3DDefinition axis)
   {
      this.axis = axis;
   }

   public void setLength(double length)
   {
      this.length = Double.toString(length);
   }

   @XmlElement
   public void setLength(String length)
   {
      this.length = length;
   }

   public void setRadius(double radius)
   {
      this.radius = Double.toString(radius);
   }

   @XmlElement
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   public YoTuple3DDefinition getCenter()
   {
      return center;
   }

   public YoTuple3DDefinition getAxis()
   {
      return axis;
   }

   public String getLength()
   {
      return length;
   }

   public String getRadius()
   {
      return radius;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (!super.equals(object))
      {
         return false;
      }
      else if (object instanceof YoGraphicCapsule3DDefinition other)
      {
         if (!Objects.equals(center, other.center))
            return false;
         if (!Objects.equals(axis, other.axis))
            return false;
         if (!Objects.equals(length, other.length))
            return false;
         if (!Objects.equals(radius, other.radius))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
