package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.function.Supplier;

import javafx.scene.paint.Color;

public abstract class YoGraphicFX3D extends YoGraphicFX
{
   public static final Color DEFAULT_COLOR = Color.BLUE;

   protected Supplier<Color> color = () -> DEFAULT_COLOR;

   public YoGraphicFX3D()
   {
   }

   public final void setColor(Supplier<Color> color)
   {
      this.color = color;
   }

   public final void setColor(Color color)
   {
      this.color = () -> color;
   }

   public final Supplier<Color> getColor()
   {
      return color;
   }

   @Override
   public void detachFromParent()
   {
      YoGroupFX parentGroup = getParentGroup();

      if (parentGroup != null)
      {
         parentGroup.removeYoGraphicFX3D(this);
         parentGroupProperty().set(null);
      }
   }
}
