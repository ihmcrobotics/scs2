package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

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

   public YoGraphicArrow3DDefinition()
   {
      registerTuple3DField("origin", this::getOrigin, this::setOrigin);
      registerTuple3DField("direction", this::getDirection, this::setDirection);
      registerField("scaleLength", this::isScaleLength, this::setScaleLength);
      registerField("bodyLength", this::getBodyLength, this::setBodyLength);
      registerField("headLength", this::getHeadLength, this::setHeadLength);
      registerField("scaleRadius", this::isScaleRadius, this::setScaleRadius);
      registerField("bodyRadius", this::getBodyRadius, this::setBodyRadius);
      registerField("headRadius", this::getHeadRadius, this::setHeadRadius);
   }

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
      else if (object instanceof YoGraphicArrow3DDefinition other)
      {
         if (!Objects.equals(origin, other.origin))
            return false;
         if (!Objects.equals(direction, other.direction))
            return false;
         if (scaleLength != other.scaleLength)
            return false;
         if (!Objects.equals(bodyLength, other.bodyLength))
            return false;
         if (!Objects.equals(headLength, other.headLength))
            return false;
         if (scaleRadius != other.scaleRadius)
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
}
