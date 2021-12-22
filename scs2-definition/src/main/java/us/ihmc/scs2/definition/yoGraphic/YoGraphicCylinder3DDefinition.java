package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicCylinder3D")
public class YoGraphicCylinder3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition center;
   private YoTuple3DDefinition axis;

   private String length;
   private String radius;

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
      else if (object instanceof YoGraphicCylinder3DDefinition)
      {
         YoGraphicCylinder3DDefinition other = (YoGraphicCylinder3DDefinition) object;

         if (center == null ? other.center != null : !center.equals(other.center))
            return false;
         if (axis == null ? other.axis != null : !axis.equals(other.axis))
            return false;
         if (length == null ? other.length != null : !length.equals(other.length))
            return false;
         if (radius == null ? other.radius != null : !radius.equals(other.radius))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return super.toString() + ", center: " + center + ", direction: " + axis + ", length: " + length + ", radius: " + radius + ", color: " + color;
   }
}
