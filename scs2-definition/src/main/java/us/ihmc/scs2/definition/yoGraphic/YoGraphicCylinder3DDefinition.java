package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

/**
 * A {@code YoGraphicCylinder3DDefinition} is a template for creating 3D cylinder and which
 * components can be backed by {@code YoVariable}s.
 * <p>
 * The {@code YoGraphicCylinder3DDefinition} is to be passed before initialization of a session
 * (either before starting a simulation or when creating a yoVariable server), such that the SCS GUI
 * can use the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicCylinder3D")
public class YoGraphicCylinder3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the center of the cylinder. */
   private YoTuple3DDefinition center;
   /** The axis of the cylinder. */
   private YoTuple3DDefinition axis;
   /** The length of the cylinder. */
   private String length;
   /** The radius of the cylinder. */
   private String radius;

   /**
    * Creates a new yoGraphic definition for rendering a cylinder.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicCylinder3DDefinition()
   {
      registerTuple3DField("center", this::getCenter, this::setCenter);
      registerTuple3DField("axis", this::getAxis, this::setAxis);
      registerStringField("length", this::getLength, this::setLength);
      registerStringField("radius", this::getRadius, this::setRadius);
   }

   /**
    * Sets the position of the center of the cylinder.
    * 
    * @param center the position of the center of the cylinder.
    */
   @XmlElement
   public void setCenter(YoTuple3DDefinition center)
   {
      this.center = center;
   }

   /**
    * Sets the axis of the cylinder.
    * 
    * @param axis the axis of the cylinder.
    */
   @XmlElement
   public void setAxis(YoTuple3DDefinition axis)
   {
      this.axis = axis;
   }

   /**
    * Sets the length of the cylinder to a constant value.
    * 
    * @param length the length of the cylinder.
    */
   public void setLength(double length)
   {
      this.length = Double.toString(length);
   }

   /**
    * Sets the length of the cylinder. It can be backed by a {@code YoVariable} by setting it to the
    * variable's name/fullname.
    * 
    * @param length the length of the cylinder.
    */
   @XmlElement
   public void setLength(String length)
   {
      this.length = length;
   }

   /**
    * Sets the radius of the cylinder to a constant value.
    * 
    * @param radius the radius of the cylinder.
    */
   public void setRadius(double radius)
   {
      this.radius = Double.toString(radius);
   }

   /**
    * Sets the radius of the cylinder. It can be backed by a {@code YoVariable} by setting it to the
    * variable's name/fullname.
    * 
    * @param cength the radius of the cylinderF.
    */
   @XmlElement
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   public YoTuple3DDefinition getCenter()
   {
      return center;
   }

   public YoTuple3DDefinition getAxis()
   {
      return axis;
   }

   public String getLength()
   {
      return length;
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
      else if (object instanceof YoGraphicCylinder3DDefinition other)
      {
         if (!Objects.equals(center, other.center))
            return false;
         if (!Objects.equals(axis, other.axis))
            return false;
         if (!Objects.equals(length, other.length))
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
