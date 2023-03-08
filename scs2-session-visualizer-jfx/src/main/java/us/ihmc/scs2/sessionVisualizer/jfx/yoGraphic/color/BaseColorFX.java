package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color;

import javafx.scene.paint.Color;

public interface BaseColorFX// extends Supplier<Color>
{
   Color get();

   void clear();

   BaseColorFX clone();
}
