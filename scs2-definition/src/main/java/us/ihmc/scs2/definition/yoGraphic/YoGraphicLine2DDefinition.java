package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicLine2D")
public class YoGraphicLine2DDefinition extends YoGraphic2DDefinition
{
   private YoTuple2DDefinition origin;
   private YoTuple2DDefinition direction;
   private YoTuple2DDefinition destination;

   @XmlElement
   public void setOrigin(YoTuple2DDefinition origin)
   {
      this.origin = origin;
   }

   @XmlElement
   public void setDirection(YoTuple2DDefinition direction)
   {
      this.direction = direction;
   }

   @XmlElement
   public void setDestination(YoTuple2DDefinition destination)
   {
      this.destination = destination;
   }

   public YoTuple2DDefinition getOrigin()
   {
      return origin;
   }

   public YoTuple2DDefinition getDirection()
   {
      return direction;
   }

   public YoTuple2DDefinition getDestination()
   {
      return destination;
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
      else if (object instanceof YoGraphicLine2DDefinition)
      {
         YoGraphicLine2DDefinition other = (YoGraphicLine2DDefinition) object;

         if (origin == null ? other.origin != null : !origin.equals(other.origin))
            return false;
         if (direction == null ? other.direction != null : !direction.equals(other.direction))
            return false;
         if (destination == null ? other.destination != null : !destination.equals(other.destination))
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
      return "origin: " + origin + (direction != null ? (", direction: " + direction) : (", destination: " + destination)) + ", fillColor: " + fillColor
            + ", strokeColor: " + strokeColor + ", strokeWidth: " + strokeWidth;
   }
}
