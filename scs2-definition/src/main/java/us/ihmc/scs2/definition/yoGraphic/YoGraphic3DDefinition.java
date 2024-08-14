package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import java.util.Objects;

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
   protected String drawMode = "FILL";

   public YoGraphic3DDefinition()
   {
   }

   public YoGraphic3DDefinition(YoGraphic3DDefinition other)
   {
      super(other);
      color = other.color == null ? null : other.color.copy();
      drawMode = other.drawMode;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerPaintField("color", this::getColor, this::setColor);
      registerStringField("drawMode", this::getDrawMode, this::setDrawMode);
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

   /**
    * Sets the draw mode of the shape.
    *
    * @param drawMode the draw mode of the shape. The following values are supported: "FILL", "LINE".
    */
   @XmlElement
   public final void setDrawMode(String drawMode)
   {
      this.drawMode = drawMode;
   }

   public final PaintDefinition getColor()
   {
      return color;
   }

   public String getDrawMode()
   {
      return drawMode;
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
         if (!Objects.equals(drawMode, other.drawMode))
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
      this.color = color;
   }

   // For backward compatibility.
   @Deprecated
   public ColorDefinition getColorOld()
   {
      return null;
   }
}
