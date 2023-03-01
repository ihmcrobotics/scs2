package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;

public abstract class YoGraphic2DDefinition extends YoGraphicDefinition
{
   protected PaintDefinition fillColor;
   protected PaintDefinition strokeColor;
   protected String strokeWidth;

   public YoGraphic2DDefinition()
   {
      registerPaintField("fillColor", this::getFillColor, this::setFillColor);
      registerPaintField("strokeColor", this::getStrokeColor, this::setStrokeColor);
      registerField("strokeWidth", this::getStrokeWidth, this::setStrokeWidth);
   }

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

   @XmlElement(name = "fillColorNew")
   public final void setFillColor(PaintDefinition fillColor)
   {
      this.fillColor = fillColor;
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

   @XmlElement(name = "strokeColorNew")
   public final void setStrokeColor(PaintDefinition strokeColor)
   {
      this.strokeColor = strokeColor;
   }

   public final void setStrokeWidth(double strokeWidth)
   {
      this.strokeWidth = Double.toString(strokeWidth);
   }

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
}
