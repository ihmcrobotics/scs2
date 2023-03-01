package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicLine2D")
public class YoGraphicLine2DDefinition extends YoGraphic2DDefinition
{
   private YoTuple2DDefinition origin;
   private YoTuple2DDefinition direction;
   private YoTuple2DDefinition destination;

   public YoGraphicLine2DDefinition()
   {
      registerTuple2DField("origin", this::getOrigin, this::setOrigin);
      registerTuple2DField("direction", this::getDirection, this::setDirection);
      registerTuple2DField("destination", this::getDestination, this::setDestination);
   }

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
      else if (object instanceof YoGraphicLine2DDefinition other)
      {
         if (!Objects.equals(origin, other.origin))
            return false;
         if (!Objects.equals(direction, other.direction))
            return false;
         if (!Objects.equals(destination, other.destination))
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
