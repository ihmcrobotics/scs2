package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

/**
 * A {@code YoGraphicCone3DDefinition} is a template for creating 3D cone and which components can
 * be backed by {@code YoVariable}s.
 * <p>
 * The {@code YoGraphicCone3DDefinition} is to be passed before initialization of a session (either
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
@XmlRootElement(name = "YoGraphicCone3D")
public class YoGraphicCone3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the base center of the cone. */
   private YoTuple3DDefinition position;
   /** The axis of the cone. */
   private YoTuple3DDefinition axis;
   /** The height of the cone. */
   private String height;
   /** The radius of the base of the cone. */
   private String radius;

   /**
    * Creates a new yoGraphic definition for rendering a cone.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicCone3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerTuple3DField("axis", this::getAxis, this::setAxis);
      registerStringField("height", this::getHeight, this::setHeight);
      registerStringField("radius", this::getRadius, this::setRadius);
   }

   /**
    * Sets the position of the base center of the cone.
    * 
    * @param position the position of the base center of the cone.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the axis of the cone.
    * 
    * @param axis the axis of the cone.
    */
   @XmlElement
   public void setAxis(YoTuple3DDefinition axis)
   {
      this.axis = axis;
   }

   /**
    * Sets the height of the cone to a constant value.
    * 
    * @param height the height of the cone.
    */
   public void setHeight(double height)
   {
      this.height = Double.toString(height);
   }

   /**
    * Sets the height of the cone. It can be backed by a {@code YoVariable} by setting it to the
    * variable's name/fullname.
    * 
    * @param height the height of the cone.
    */
   @XmlElement
   public void setHeight(String height)
   {
      this.height = height;
   }

   /**
    * Sets the radius of the cone to a constant value.
    * 
    * @param radius the radius of the cone.
    */

   public void setRadius(double radius)
   {
      this.radius = Double.toString(radius);
   }

   /**
    * Sets the radius of the cone. It can be backed by a {@code YoVariable} by setting it to the
    * variable's name/fullname.
    * 
    * @param cength the radius of the cone.
    */
   @XmlElement
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoTuple3DDefinition getAxis()
   {
      return axis;
   }

   public String getHeight()
   {
      return height;
   }

   public String getRadius()
   {
      return radius;
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
      else if (object instanceof YoGraphicCone3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(axis, other.axis))
            return false;
         if (!Objects.equals(height, other.height))
            return false;
         if (!Objects.equals(radius, other.radius))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
