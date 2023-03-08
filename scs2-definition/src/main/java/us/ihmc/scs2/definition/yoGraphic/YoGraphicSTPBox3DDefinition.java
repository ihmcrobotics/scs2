package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * A {@code YoGraphicSTPBox3DDefinition} is a template for creating a 3D STP box and which
 * components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoSTPBoxFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicSTPBox3DDefinition} is to be passed before initialization of a session
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
@XmlRootElement(name = "YoGraphicSTPBox3D")
public class YoGraphicSTPBox3DDefinition extends YoGraphicBox3DDefinition
{
   /** The minimum distance between the original shape and the STP shape. */
   private String minimumMargin;
   /** The maximum distance between the original shape and the STP shape. */
   private String maximumMargin;

   /**
    * Creates a new yoGraphic definition for rendering a STP box.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicSTPBox3DDefinition()
   {
      registerStringField("minimumMargin", this::getMinimumMargin, this::setMinimumMargin);
      registerStringField("maximumMargin", this::getMaximumMargin, this::setMaximumMargin);
   }

   /**
    * Sets the minimum margin for the STP box to a constant value.
    * 
    * @param minimumMargin the minimum margin.
    */
   public void setMinimumMargin(double minimumMargin)
   {
      this.minimumMargin = Double.toString(minimumMargin);
   }

   /**
    * Sets the minimum margin for the STP box.
    * <p>
    * Using this method allows to back the margin with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    * 
    * @param minimumMargin the minimum margin.
    */
   @XmlElement
   public void setMinimumMargin(String minimumMargin)
   {
      this.minimumMargin = minimumMargin;
   }

   /**
    * Sets the maximum margin for the STP box to a constant value.
    * 
    * @param maximumMargin the maximum margin.
    */
   public void setMaximumMargin(double maximumMargin)
   {
      this.maximumMargin = Double.toString(maximumMargin);
   }

   /**
    * Sets the maximum margin for the STP box.
    * <p>
    * Using this method allows to back the margin with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    * 
    * @param maximumMargin the maximum margin.
    */
   @XmlElement
   public void setMaximumMargin(String maximumMargin)
   {
      this.maximumMargin = maximumMargin;
   }

   public String getMinimumMargin()
   {
      return minimumMargin;
   }

   public String getMaximumMargin()
   {
      return maximumMargin;
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
      else if (object instanceof YoGraphicSTPBox3DDefinition other)
      {
         if (!Objects.equals(minimumMargin, other.minimumMargin))
            return false;
         if (!Objects.equals(maximumMargin, other.maximumMargin))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
