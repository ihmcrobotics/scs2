package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicCone3D")
public class YoGraphicCone3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoTuple3DDefinition axis;

   private String height;
   private String radius;

   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   @XmlElement
   public void setAxis(YoTuple3DDefinition axis)
   {
      this.axis = axis;
   }

   public void setHeight(double height)
   {
      this.height = Double.toString(height);
   }

   @XmlElement
   public void setHeight(String height)
   {
      this.height = height;
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

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoTuple3DDefinition getAxis()
   {
      return axis;
   }

   public String getHeight()
   {
      return height;
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
      else if (object instanceof YoGraphicCone3DDefinition)
      {
         YoGraphicCone3DDefinition other = (YoGraphicCone3DDefinition) object;

         if (position == null ? other.position != null : !position.equals(other.position))
            return false;
         if (axis == null ? other.axis != null : !axis.equals(other.axis))
            return false;
         if (height == null ? other.height != null : !height.equals(other.height))
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
      return super.toString() + ", position: " + position + ", direction: " + axis + ", height: " + height + ", radius: " + radius + ", color: " + color;
   }
}
