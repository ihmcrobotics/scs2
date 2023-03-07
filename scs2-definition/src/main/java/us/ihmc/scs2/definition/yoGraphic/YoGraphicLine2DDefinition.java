package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

/**
 * A {@code YoGraphicLine2DDefinition} is a template for creating 2D line and which components can
 * be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoLineFX2D.png"
 * width=150px/>
 * <p>
 * A 2D yoGraphic is rendered in the overhead plotter panel and it can be back by
 * {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphicLine2DDefinition} is to be passed before initialization of a session (either
 * before starting a simulation or when creating a yoVariable server), such that the SCS GUI can use
 * the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicLine2D")
public class YoGraphicLine2DDefinition extends YoGraphic2DDefinition
{
   /** The line's origin. */
   private YoTuple2DDefinition origin;
   /** The line's direction vector, {@code null} if the {@link #destination} is to be used instead. */
   private YoTuple2DDefinition direction;
   /** The line's destination point, {@code null} if the {@link #direction} is to be used instead. */
   private YoTuple2DDefinition destination;

   /**
    * Creates a new yoGraphic definition for rendering a line.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicLine2DDefinition()
   {
      registerTuple2DField("origin", this::getOrigin, this::setOrigin);
      registerTuple2DField("direction", this::getDirection, this::setDirection);
      registerTuple2DField("destination", this::getDestination, this::setDestination);
   }

   /**
    * Sets the line origin.
    * 
    * @param origin the position of the line origin.
    */
   @XmlElement
   public void setOrigin(YoTuple2DDefinition origin)
   {
      this.origin = origin;
   }

   /**
    * Sets the line direction.
    * <p>
    * The length of the line is set to the direction's magnitude.
    * </p>
    * 
    * @param direction the vector for the line direction, or {@code null} if the destination is to be
    *                  used instead.
    */
   @XmlElement
   public void setDirection(YoTuple2DDefinition direction)
   {
      this.direction = direction;
   }

   /**
    * Sets the line destination.
    * 
    * @param destination the position for the line destination, or {@code null} if the direction is to
    *                    be used instead.
    */
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
}
