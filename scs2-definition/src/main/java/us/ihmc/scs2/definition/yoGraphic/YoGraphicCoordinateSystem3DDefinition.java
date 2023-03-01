package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicCoordinateSystem3D")
public class YoGraphicCoordinateSystem3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoOrientation3DDefinition orientation;
   private String bodyLength;
   private String headLength;
   private String bodyRadius;
   private String headRadius;

   public YoGraphicCoordinateSystem3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerField("bodyLength", this::getBodyLength, this::setBodyLength);
      registerField("headLength", this::getHeadLength, this::setHeadLength);
      registerField("bodyRadius", this::getBodyRadius, this::setBodyRadius);
      registerField("headRadius", this::getHeadRadius, this::setHeadRadius);
   }

   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   public void setBodyLength(double bodyLength)
   {
      this.bodyLength = Double.toString(bodyLength);
   }

   @XmlElement
   public void setBodyLength(String bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   public void setBodyRadius(double bodyRadius)
   {
      this.bodyRadius = Double.toString(bodyRadius);
   }

   @XmlElement
   public void setBodyRadius(String bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   public void setHeadLength(double headLength)
   {
      this.headLength = Double.toString(headLength);
   }

   @XmlElement
   public void setHeadLength(String headLength)
   {
      this.headLength = headLength;
   }

   public void setHeadRadius(double headRadius)
   {
      this.headRadius = Double.toString(headRadius);
   }

   @XmlElement
   public void setHeadRadius(String headRadius)
   {
      this.headRadius = headRadius;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public String getBodyLength()
   {
      return bodyLength;
   }

   public String getHeadLength()
   {
      return headLength;
   }

   public String getBodyRadius()
   {
      return bodyRadius;
   }

   public String getHeadRadius()
   {
      return headRadius;
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
      else if (object instanceof YoGraphicCoordinateSystem3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(orientation, other.orientation))
            return false;
         if (!Objects.equals(bodyLength, other.bodyLength))
            return false;
         if (!Objects.equals(headLength, other.headLength))
            return false;
         if (!Objects.equals(bodyRadius, other.bodyRadius))
            return false;
         if (!Objects.equals(headRadius, other.headRadius))
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
      return super.toString() + ", position: " + position + ", orientation: " + orientation + ", bodyLength: " + bodyLength + ", headLength: " + headLength
            + ", bodyRadius: " + bodyRadius + ", headRadius: " + headRadius + ", color: " + color;
   }
}
