package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

/**
 * A {@code YoGraphicEllipsoid3DDefinition} is a template for creating 3D ellipsoid and which
 * components can be backed by {@code YoVariable}s.
 * <p>
 * The {@code YoGraphicEllipsoid3DDefinition} is to be passed before initialization of a session
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
@XmlRootElement(name = "YoGraphicEllipsoid3D")
public class YoGraphicEllipsoid3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the center of the ellipsoid. */
   private YoTuple3DDefinition position;
   /** The orientation of the ellipsoid. */
   private YoOrientation3DDefinition orientation;
   /** The radii of the ellipsoid. */
   private YoTuple3DDefinition radii;

   /**
    * Creates a new yoGraphic definition for rendering a ellipsoid.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicEllipsoid3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerTuple3DField("radii", this::getRadii, this::setRadii);
   }

   /**
    * Sets the position of the center of the ellipsoid.
    * 
    * @param position the position of the center of the ellipsoid.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation of the ellipsoid.
    * 
    * @param orientation the orientation of the ellipsoid.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the radii of the ellipsoid.
    * 
    * @param radii the radii of the ellipsoid.
    */
   @XmlElement
   public void setRadii(YoTuple3DDefinition radii)
   {
      this.radii = radii;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public YoTuple3DDefinition getRadii()
   {
      return radii;
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
      else if (object instanceof YoGraphicEllipsoid3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(orientation, other.orientation))
            return false;
         if (!Objects.equals(radii, other.radii))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
