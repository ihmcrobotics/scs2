package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.Objects;

/**
 * A {@code YoGraphicBox3DDefinition} is a template for creating 3D box and which components can be
 * backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoBoxFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicBox3DDefinition} is to be passed before initialization of a session (either
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
@XmlRootElement(name = "YoGraphicBox3D")
public class YoGraphicBox3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the center of the box. */
   private YoTuple3DDefinition position;
   /** The orientation of the box. */
   private YoOrientation3DDefinition orientation;
   /** The size of the box. */
   private YoTuple3DDefinition size;

   /**
    * Creates a new yoGraphic definition for rendering a box.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicBox3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicBox3DDefinition(YoGraphicBox3DDefinition other)
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
    * Sets the position of the center of the box.
    *
    * @param position the position of the center of the box.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation of the box.
    *
    * @param orientation the orientation of the box.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the size of the box.
    *
    * @param size the size of the box.
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
   public YoGraphicBox3DDefinition copy()
   {
      return new YoGraphicBox3DDefinition(this);
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
      else if (object instanceof YoGraphicBox3DDefinition other)
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
