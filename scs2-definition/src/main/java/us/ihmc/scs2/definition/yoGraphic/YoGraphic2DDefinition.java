package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;

import javax.xml.bind.annotation.XmlElement;
import java.util.Objects;

/**
 * Base class representing a template used to create a single 2D yoGraphic.
 * <p>
 * A 2D yoGraphic is rendered in the overhead plotter panel and it can be back by
 * {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphic2DDefinition} is to be passed before initialization of a session (either
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
public abstract class YoGraphic2DDefinition extends YoGraphicDefinition
{
   /** The color to fill the shape with or {@code null} for no fill. */
   protected PaintDefinition fillColor;
   /** The color of the stroke or {@code null} for no stroke. */
   protected PaintDefinition strokeColor;
   /** The width of the stroke. */
   protected String strokeWidth;

   public YoGraphic2DDefinition()
   {
      super();
   }

   /**
    * Creates a new 2D yoGraphic definition.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphic2DDefinition(YoGraphic2DDefinition other)
   {
      super(other);
      fillColor = other.fillColor == null ? null : other.fillColor.copy();
      strokeColor = other.strokeColor == null ? null : other.strokeColor.copy();
      strokeWidth = other.strokeWidth;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerPaintField("fillColor", this::getFillColor, this::setFillColor);
      registerPaintField("strokeColor", this::getStrokeColor, this::setStrokeColor);
      registerStringField("strokeWidth", this::getStrokeWidth, this::setStrokeWidth);
   }

   /**
    * Sets the color to fill the shape with or {@code null} for no fill.
    * <p>
    * See {@link ColorDefinition} for setting the color to a constant value, or other implementations
    * of {@link PaintDefinition} notably for colors backed by {@code YoVariable}s.
    * </p>
    *
    * @param fillColor the color to fill the shape with.
    */
   @XmlElement(name = "fillColorNew")
   public final void setFillColor(PaintDefinition fillColor)
   {
      this.fillColor = fillColor;
   }

   /**
    * Sets the color of the shape's stroke or {@code null} for no stroke.
    * <p>
    * See {@link ColorDefinition} for setting the color to a constant value, or other implementations
    * of {@link PaintDefinition} notably for colors backed by {@code YoVariable}s.
    * </p>
    *
    * @param strokeColor the stroke color.
    */
   @XmlElement(name = "strokeColorNew")
   public final void setStrokeColor(PaintDefinition strokeColor)
   {
      this.strokeColor = strokeColor;
   }

   /**
    * Sets a constant value the width of the stroke.
    *
    * @param strokeWidth the stroke width.
    */
   public final void setStrokeWidth(double strokeWidth)
   {
      this.strokeWidth = Double.toString(strokeWidth);
   }

   /**
    * Sets the width of the stroke, can be backed by a {@code YoVariable} by providing the
    * name/fullname, or a constant.
    *
    * @param strokeWidth the stroke width.
    */
   @XmlElement
   public final void setStrokeWidth(String strokeWidth)
   {
      this.strokeWidth = strokeWidth;
   }

   public final PaintDefinition getFillColor()
   {
      return fillColor;
   }

   public final PaintDefinition getStrokeColor()
   {
      return strokeColor;
   }

   public final String getStrokeWidth()
   {
      return strokeWidth;
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
      else if (object instanceof YoGraphic2DDefinition other)
      {
         if (!Objects.equals(fillColor, other.fillColor))
            return false;
         if (!Objects.equals(strokeColor, other.strokeColor))
            return false;
         if (!Objects.equals(strokeWidth, other.strokeWidth))
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
   @XmlElement(name = "fillColor")
   public final void setFillColorOld(ColorDefinition fillColor)
   {
      this.fillColor = fillColor;
   }

   // For backward compatibility.
   @Deprecated
   public ColorDefinition getFillColorOld()
   {
      return null;
   }

   // For backward compatibility.
   @Deprecated
   @XmlElement(name = "strokeColor")
   public final void setStrokeColorOld(ColorDefinition strokeColor)
   {
      this.strokeColor = strokeColor;
   }

   // For backward compatibility.
   @Deprecated
   public ColorDefinition getStrokeColorOld()
   {
      return null;
   }
}
