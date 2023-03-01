package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

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

   public YoGraphicCone3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerTuple3DField("axis", this::getAxis, this::setAxis);
      registerField("height", this::getHeight, this::setHeight);
      registerField("radius", this::getRadius, this::setRadius);
   }

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
      else if (object instanceof YoGraphicCone3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(axis, other.axis))
            return false;
         if (!Objects.equals(height, other.height))
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
