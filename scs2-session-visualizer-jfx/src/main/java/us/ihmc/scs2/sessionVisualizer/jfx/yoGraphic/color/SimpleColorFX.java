package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color;

import javafx.scene.paint.Color;

public class SimpleColorFX implements BaseColorFX
{
   private Color color = null;

   public SimpleColorFX()
   {
   }

   public SimpleColorFX(Color color)
   {
      this.color = color;
   }

   public SimpleColorFX(SimpleColorFX other)
   {
      this.color = other.color;
   }

   @Override
   public void clear()
   {
      color = null;
   }

   @Override
   public Color get()
   {
      return color;
   }

   public final double getRed()
   {
      return color.getRed();
   }

   public final double getGreen()
   {
      return color.getGreen();
   }

   public final double getBlue()
   {
      return color.getBlue();
   }

   public final double getOpacity()
   {
      return color.getOpacity();
   }

   @Override
   public SimpleColorFX clone()
   {
      return new SimpleColorFX(this);
   }

   @Override
   public String toString()
   {
      return "[color=" + color + "]";
   }
}
