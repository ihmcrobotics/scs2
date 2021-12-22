package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicArrow3D")
public class YoGraphicArrow3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition origin;
   private YoTuple3DDefinition direction;

   private boolean scaleLength;
   private String bodyLength;
   private String headLength;
   private boolean scaleRadius;
   private String bodyRadius;
   private String headRadius;

   @XmlElement
   public void setOrigin(YoTuple3DDefinition origin)
   {
      this.origin = origin;
   }

   @XmlElement
   public void setDirection(YoTuple3DDefinition direction)
   {
      this.direction = direction;
   }

   @XmlElement
   public void setScaleLength(boolean scaleLength)
   {
      this.scaleLength = scaleLength;
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

   @XmlElement
   public void setScaleRadius(boolean scaleRadius)
   {
      this.scaleRadius = scaleRadius;
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

   public YoTuple3DDefinition getOrigin()
   {
      return origin;
   }

   public YoTuple3DDefinition getDirection()
   {
      return direction;
   }

   public boolean isScaleLength()
   {
      return scaleLength;
   }

   public String getBodyLength()
   {
      return bodyLength;
   }

   public String getHeadLength()
   {
      return headLength;
   }

   public boolean isScaleRadius()
   {
      return scaleRadius;
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
      else if (object instanceof YoGraphicArrow3DDefinition)
      {
         YoGraphicArrow3DDefinition other = (YoGraphicArrow3DDefinition) object;

         if (origin == null ? other.origin != null : !origin.equals(other.origin))
            return false;
         if (direction == null ? other.direction != null : !direction.equals(other.direction))
            return false;
         if (scaleLength != other.scaleLength)
            return false;
         if (bodyLength == null ? other.bodyLength != null : !bodyLength.equals(other.bodyLength))
            return false;
         if (headLength == null ? other.headLength != null : !headLength.equals(other.headLength))
            return false;
         if (scaleRadius != other.scaleRadius)
            return false;
         if (bodyRadius == null ? other.bodyRadius != null : !bodyRadius.equals(other.bodyRadius))
            return false;
         if (headRadius == null ? other.headRadius != null : !headRadius.equals(other.headRadius))
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
      return super.toString() + ", origin: " + origin + ", direction: " + direction + ", scale length: " + scaleLength + ", scale radius: " + scaleRadius
            + ", bodyLength: " + bodyLength + ", headLength: " + headLength + ", bodyRadius: " + bodyRadius + ", headRadius: " + headRadius + ", color: "
            + color;
   }
}
