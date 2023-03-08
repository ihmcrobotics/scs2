package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color;

import javafx.beans.property.IntegerProperty;
import javafx.scene.paint.Color;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;

public class YoColorRGBASingleFX implements BaseColorFX
{
   private IntegerProperty rgba;

   public YoColorRGBASingleFX()
   {
   }

   public YoColorRGBASingleFX(IntegerProperty rgba)
   {
      this.rgba = rgba;
   }

   public YoColorRGBASingleFX(YoColorRGBASingleFX other)
   {
      rgba = other.rgba;
   }

   @Override
   public void clear()
   {
      rgba = null;
   }

   public void setRGBA(IntegerProperty rgba)
   {
      this.rgba = rgba;
   }

   public IntegerProperty getRGBA()
   {
      return rgba;
   }

   @Override
   public YoColorRGBASingleFX clone()
   {
      return new YoColorRGBASingleFX(this);
   }

   @Override
   public Color get()
   {
      return rgba == null ? null : JavaFXVisualTools.toColor(ColorDefinitions.rgba(rgba.get()));
   }

   @Override
   public String toString()
   {
      return "[rgba=" + rgba + "]";
   }
}