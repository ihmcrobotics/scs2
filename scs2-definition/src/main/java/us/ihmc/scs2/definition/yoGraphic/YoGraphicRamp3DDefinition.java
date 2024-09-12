package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.Objects;

@XmlRootElement(name = "YoGraphicRamp3D")
public class YoGraphicRamp3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the center of the ramp. */
   private YoTuple3DDefinition position;
   /** The orientation of the ramp. */
   private YoOrientation3DDefinition orientation;
   /** The size of the ramp. */
   private YoTuple3DDefinition size;

   /**
    * Creates a new yoGraphic definition for rendering a ramp.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicRamp3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicRamp3DDefinition(YoGraphicRamp3DDefinition other)
   {
      super(other);
      position = other.position == null ? null : other.position.copy();
      orientation = other.orientation == null ? null : other.orientation.copy();
      size = other.size == null ? null : other.size.copy();
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerTuple3DField("size", this::getSize, this::setSize);
   }

   /**
    * Sets the position of the center of the ramp.
    *
    * @param position the position of the center of the ramp.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation of the ramp.
    *
    * @param orientation the orientation of the ramp.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the size of the ramp.
    *
    * @param size the size of the ramp.
    */
   @XmlElement
   public void setSize(YoTuple3DDefinition size)
   {
      this.size = size;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public YoTuple3DDefinition getSize()
   {
      return size;
   }

   @Override
   public YoGraphicRamp3DDefinition copy()
   {
      return new YoGraphicRamp3DDefinition(this);
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
      else if (object instanceof YoGraphicRamp3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(orientation, other.orientation))
            return false;
         if (!Objects.equals(size, other.size))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
