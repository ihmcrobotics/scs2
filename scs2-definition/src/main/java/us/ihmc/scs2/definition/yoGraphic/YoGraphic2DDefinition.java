package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.visual.ColorDefinition;

public abstract class YoGraphic2DDefinition extends YoGraphicDefinition
{
   protected ColorDefinition fillColor;
   protected ColorDefinition strokeColor;
   protected String strokeWidth;

   @XmlElement
   public final void setFillColor(ColorDefinition fillColor)
   {
      this.fillColor = fillColor;
   }

   @XmlElement
   public final void setStrokeColor(ColorDefinition strokeColor)
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

   public final ColorDefinition getFillColor()
   {
      return fillColor;
   }

   public final ColorDefinition getStrokeColor()
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
      else if (object instanceof YoGraphic2DDefinition)
      {
         YoGraphic2DDefinition other = (YoGraphic2DDefinition) object;

         if (fillColor == null ? other.fillColor != null : !fillColor.equals(other.fillColor))
            return false;
         if (strokeColor == null ? other.strokeColor != null : !strokeColor.equals(other.strokeColor))
            return false;
         if (strokeWidth == null ? other.strokeWidth != null : !strokeWidth.equals(other.strokeWidth))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
