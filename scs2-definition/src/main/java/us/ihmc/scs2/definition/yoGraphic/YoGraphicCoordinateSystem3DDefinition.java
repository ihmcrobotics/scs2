package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

/**
 * A {@code YoGraphicCoordinateSystem3DDefinition} is a template for creating 3D coordinate system
 * and which components can be backed by {@code YoVariable}s.
 * <p>
 * The {@code YoGraphicCoordinateSystem3DDefinition} is to be passed before initialization of a
 * session (either before starting a simulation or when creating a yoVariable server), such that the
 * SCS GUI can use the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicCoordinateSystem3D")
public class YoGraphicCoordinateSystem3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the coordinate system. */
   private YoTuple3DDefinition position;
   /** The orientation of the coordinate system. */
   private YoOrientation3DDefinition orientation;
   /** The length of the body part for each arrow. */
   private String bodyLength;
   /** The length of the head part for each arrow. */
   private String headLength;
   /** The radius of the body part for each arrow. */
   private String bodyRadius;
   /** The radius of the head part for each arrow. */
   private String headRadius;

   /**
    * Creates a new yoGraphic definition for rendering an coordinate system.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicCoordinateSystem3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerStringField("bodyLength", this::getBodyLength, this::setBodyLength);
      registerStringField("headLength", this::getHeadLength, this::setHeadLength);
      registerStringField("bodyRadius", this::getBodyRadius, this::setBodyRadius);
      registerStringField("headRadius", this::getHeadRadius, this::setHeadRadius);
   }

   /**
    * Sets the position of the coordinate system.
    * 
    * @param position the position of the coordinate system.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation of the coordinate system.
    * 
    * @param orientation the orientation of the coordinate system.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the length of the body part for each arrow to a constant value.
    * 
    * @param bodyLength the length of the body part for each arrow.
    */
   public void setBodyLength(double bodyLength)
   {
      this.bodyLength = Double.toString(bodyLength);
   }

   /**
    * Sets the length of the body part for each arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    * 
    * @param bodyLength the length of the body part for each arrow.
    */
   @XmlElement
   public void setBodyLength(String bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   /**
    * Sets the length of the head part for each arrow to a constant value.
    * 
    * @param headLength the length of the head part for each arrow.
    */
   public void setHeadLength(double headLength)
   {
      this.headLength = Double.toString(headLength);
   }

   /**
    * Sets the length of the head part for each arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    * 
    * @param headLength the length of the head part for each arrow.
    */
   @XmlElement
   public void setHeadLength(String headLength)
   {
      this.headLength = headLength;
   }

   /**
    * Sets the radius of the body part for each arrow to a constant value.
    * 
    * @param bodyRadius the radius of the body part for each arrow.
    */
   public void setBodyRadius(double bodyRadius)
   {
      this.bodyRadius = Double.toString(bodyRadius);
   }

   /**
    * Sets the radius of the body part for each arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    * 
    * @param bodyRadius the radius of the body part for each arrow.
    */
   @XmlElement
   public void setBodyRadius(String bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   /**
    * Sets the radius of the head part for each arrow to a constant value.
    * 
    * @param headRadius the radius of the head part for each arrow.
    */
   public void setHeadRadius(double headRadius)
   {
      this.headRadius = Double.toString(headRadius);
   }

   /**
    * Sets the radius of the head part for each arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    * 
    * @param headRadius the radius of the head part for each arrow.
    */
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
}
