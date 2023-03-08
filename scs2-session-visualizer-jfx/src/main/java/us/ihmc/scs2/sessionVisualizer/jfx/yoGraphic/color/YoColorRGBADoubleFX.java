package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color;

import javafx.beans.property.DoubleProperty;
import javafx.scene.paint.Color;
import us.ihmc.commons.MathTools;

public class YoColorRGBADoubleFX implements BaseColorFX
{
   private DoubleProperty red, green, blue, alpha;

   public YoColorRGBADoubleFX()
   {
   }

   public YoColorRGBADoubleFX(DoubleProperty red, DoubleProperty green, DoubleProperty blue)
   {
      this(red, green, blue, null);
   }

   public YoColorRGBADoubleFX(DoubleProperty red, DoubleProperty green, DoubleProperty blue, DoubleProperty alpha)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.alpha = alpha;
   }

   public YoColorRGBADoubleFX(YoColorRGBADoubleFX other)
   {
      this.red = other.red;
      this.green = other.green;
      this.blue = other.blue;
      this.alpha = other.alpha;
   }

   @Override
   public void clear()
   {
      this.red = null;
      this.green = null;
      this.blue = null;
      this.alpha = null;
   }

   public void setRed(DoubleProperty red)
   {
      this.red = red;
   }

   public void setGreen(DoubleProperty green)
   {
      this.green = green;
   }

   public void setBlue(DoubleProperty blue)
   {
      this.blue = blue;
   }

   public void setAlpha(DoubleProperty alpha)
   {
      this.alpha = alpha;
   }

   public DoubleProperty getRed()
   {
      return red;
   }

   public DoubleProperty getGreen()
   {
      return green;
   }

   public DoubleProperty getBlue()
   {
      return blue;
   }

   public DoubleProperty getAlpha()
   {
      return alpha;
   }

   @Override
   public YoColorRGBADoubleFX clone()
   {
      return new YoColorRGBADoubleFX(this);
   }

   @Override
   public Color get()
   {
      if (red == null && green == null && blue == null && alpha == null)
         return null;

      double r = red == null ? 0.0 : MathTools.clamp(red.get(), 0.0, 1.0);
      double g = green == null ? 0.0 : MathTools.clamp(green.get(), 0.0, 1.0);
      double b = blue == null ? 0.0 : MathTools.clamp(blue.get(), 0.0, 1.0);
      double o = alpha == null ? 1.0 : MathTools.clamp(alpha.get(), 0.0, 1.0);
      return Color.color(r, g, b, o);
   }

   @Override
   public String toString()
   {
      return "[red=" + red + ", green=" + green + ", blue=" + blue + ", alpha=" + alpha + "]";
   }
}
