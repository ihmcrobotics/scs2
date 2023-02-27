package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color;

import javafx.beans.property.IntegerProperty;
import javafx.scene.paint.Color;
import us.ihmc.commons.MathTools;

public class YoColorRGBAIntFX implements BaseColorFX
{
   private IntegerProperty red, green, blue, alpha;

   public YoColorRGBAIntFX()
   {
   }

   public YoColorRGBAIntFX(IntegerProperty red, IntegerProperty green, IntegerProperty blue)
   {
      this(red, green, blue, null);
   }

   public YoColorRGBAIntFX(IntegerProperty red, IntegerProperty green, IntegerProperty blue, IntegerProperty alpha)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.alpha = alpha;
   }

   public YoColorRGBAIntFX(YoColorRGBAIntFX other)
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

   public void setRed(IntegerProperty red)
   {
      this.red = red;
   }

   public void setGreen(IntegerProperty green)
   {
      this.green = green;
   }

   public void setBlue(IntegerProperty blue)
   {
      this.blue = blue;
   }

   public void setAlpha(IntegerProperty alpha)
   {
      this.alpha = alpha;
   }

   public IntegerProperty getRed()
   {
      return red;
   }

   public IntegerProperty getGreen()
   {
      return green;
   }

   public IntegerProperty getBlue()
   {
      return blue;
   }

   public IntegerProperty getAlpha()
   {
      return alpha;
   }

   @Override
   public YoColorRGBAIntFX clone()
   {
      return new YoColorRGBAIntFX(this);
   }

   @Override
   public Color get()
   {
      if (red == null && green == null && blue == null && alpha == null)
         return null;

      int r = red == null ? 0 : MathTools.clamp(red.get(), 0, 255);
      int g = green == null ? 0 : MathTools.clamp(green.get(), 0, 255);
      int b = blue == null ? 0 : MathTools.clamp(blue.get(), 0, 255);
      double o = alpha == null ? 1.0 : MathTools.clamp(alpha.get() / 255.0, 0.0, 1.0);
      return Color.rgb(r, g, b, o);
   }

   @Override
   public String toString()
   {
      return "[red=" + red + ", green=" + green + ", blue=" + blue + ", alpha=" + alpha + "]";
   }
}