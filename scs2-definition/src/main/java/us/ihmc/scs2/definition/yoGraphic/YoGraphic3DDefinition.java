package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;

/**
 * Base class representing a template used to create a single 3D yoGraphic.
 * <p>
 * A 3D yoGraphic is rendered in the 3D viewport together with the robot and visual definitions. It
 * can be back by {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphic3DDefinition} is to be passed before initialization of a session (either
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
public abstract class YoGraphic3DDefinition extends YoGraphicDefinition
{
   /** The diffuse color of the shape. */
   protected PaintDefinition color;

   public YoGraphic3DDefinition()
   {
      registerPaintField("color", this::getColor, this::setColor);
   }

   /**
    * Sets the diffuse color of the shape.
    * <p>
    * See {@link ColorDefinition} for setting the color to a constant value, or other implementations
    * of {@link PaintDefinition} notably for colors backed by {@code YoVariable}s.
    * </p>
    * 
    * @param color
    */
   @XmlElement(name = "colorNew")
   public final void setColor(PaintDefinition color)
   {
      this.color = color;
   }

   public final PaintDefinition getColor()
   {
      return color;
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
      else if (object instanceof YoGraphic3DDefinition other)
      {
         if (!Objects.equals(color, other.color))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   ///////////////////////////////////////////////////////////////////
   ///////////// API for backward compatibility with XML /////////////
   ///////////////////////////////////////////////////////////////////

   // For backward compatibility.
   @Deprecated
   @XmlElement(name = "color")
   public void setColorOld(ColorDefinition color)
   {
      LogTools.info("Setting color {}", color);
      this.color = color;
   }

   // For backward compatibility.
   @Deprecated
   public ColorDefinition getColorOld()
   {
      return null;
   }
}
