package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.Objects;

/**
 * A {@code YoGraphicArrow3DDefinition} is a template for creating 3D arrow/vector and which
 * components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoArrowFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicArrow3DDefinition} is to be passed before initialization of a session (either
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
@XmlRootElement(name = "YoGraphicArrow3D")
public class YoGraphicArrow3DDefinition extends YoGraphic3DDefinition
{
   /** The tuple 3D representing the origin of the arrow. */
   private YoTuple3DDefinition origin;
   /** The tuple 3D representing the direction of the arrow. */
   private YoTuple3DDefinition direction;

   /** Whether the length of the arrow should scale with the magnitude of the direction. */
   private boolean scaleLength;
   /** The length of the body part of the arrow. */
   private String bodyLength;
   /** The length of the head part of the arrow. */
   private String headLength;
   /** Whether the radius of the arrow should scale with the magnitude of the direction. */
   private boolean scaleRadius;
   /** The radius of the body part of the arrow. */
   private String bodyRadius;
   /** The radius of the head part of the arrow. */
   private String headRadius;

   /**
    * Creates a new yoGraphic definition for rendering an arrow.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicArrow3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicArrow3DDefinition(YoGraphicArrow3DDefinition other)
   {
      super(other);
      origin = other.origin == null ? null : other.origin.copy();
      direction = other.direction == null ? null : other.direction.copy();
      scaleLength = other.scaleLength;
      bodyLength = other.bodyLength;
      headLength = other.headLength;
      scaleRadius = other.scaleRadius;
      bodyRadius = other.bodyRadius;
      headRadius = other.headRadius;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerTuple3DField("origin", this::getOrigin, this::setOrigin);
      registerTuple3DField("direction", this::getDirection, this::setDirection);
      registerBooleanField("scaleLength", this::isScaleLength, this::setScaleLength);
      registerStringField("bodyLength", this::getBodyLength, this::setBodyLength);
      registerStringField("headLength", this::getHeadLength, this::setHeadLength);
      registerBooleanField("scaleRadius", this::isScaleRadius, this::setScaleRadius);
      registerStringField("bodyRadius", this::getBodyRadius, this::setBodyRadius);
      registerStringField("headRadius", this::getHeadRadius, this::setHeadRadius);
   }

   /**
    * Sets the origin of the arrow.
    *
    * @param origin the origin of the arrow.
    */
   @XmlElement
   public void setOrigin(YoTuple3DDefinition origin)
   {
      this.origin = origin;
   }

   /**
    * Sets the direction of the arrow.
    *
    * @param direction the direction of the arrow.
    */
   @XmlElement
   public void setDirection(YoTuple3DDefinition direction)
   {
      this.direction = direction;
   }

   /**
    * Sets whether the length of the arrow should scale with the direction's magnitude.
    *
    * @param scaleLength {@code true} to scale with the direction's magnitude, {@code false} for
    *                    constant length.
    */
   @XmlElement
   public void setScaleLength(boolean scaleLength)
   {
      this.scaleLength = scaleLength;
   }

   /**
    * Sets the length of the body part of the arrow to a constant value.
    *
    * @param bodyLength the length of the body part of the arrow.
    */
   public void setBodyLength(double bodyLength)
   {
      this.bodyLength = Double.toString(bodyLength);
   }

   /**
    * Sets the length of the body part of the arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    *
    * @param bodyLength the length of the body part of the arrow.
    */
   @XmlElement
   public void setBodyLength(String bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   /**
    * Sets the length of the head part of the arrow to a constant value.
    *
    * @param headLength the length of the head part of the arrow.
    */
   public void setHeadLength(double headLength)
   {
      this.headLength = Double.toString(headLength);
   }

   /**
    * Sets the length of the head part of the arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    *
    * @param headLength the length of the head part of the arrow.
    */
   @XmlElement
   public void setHeadLength(String headLength)
   {
      this.headLength = headLength;
   }

   /**
    * Sets whether the radius of the arrow should scale with the direction's magnitude.
    *
    * @param scaleRadius {@code true} to scale with the direction's magnitude, {@code false} for
    *                    constant radius.
    */
   @XmlElement
   public void setScaleRadius(boolean scaleRadius)
   {
      this.scaleRadius = scaleRadius;
   }

   /**
    * Sets the radius of the body part of the arrow to a constant value.
    *
    * @param bodyRadius the radius of the body part of the arrow.
    */
   public void setBodyRadius(double bodyRadius)
   {
      this.bodyRadius = Double.toString(bodyRadius);
   }

   /**
    * Sets the radius of the body part of the arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    *
    * @param bodyRadius the radius of the body part of the arrow.
    */
   @XmlElement
   public void setBodyRadius(String bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   /**
    * Sets the radius of the head part of the arrow to a constant value.
    *
    * @param headRadius the radius of the head part of the arrow.
    */
   public void setHeadRadius(double headRadius)
   {
      this.headRadius = Double.toString(headRadius);
   }

   /**
    * Sets the radius of the head part of the arrow. It can be backed by a {@code YoVariable} by
    * setting it to the variable's name/fullname.
    *
    * @param headRadius the radius of the head part of the arrow.
    */
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
   public YoGraphicArrow3DDefinition copy()
   {
      return new YoGraphicArrow3DDefinition(this);
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
