package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public abstract class YoGraphicFX3D extends YoGraphicFX
{
   public static final Color DEFAULT_COLOR = Color.BLUE;

   protected BaseColorFX color = new SimpleColorFX(DEFAULT_COLOR);

   public YoGraphicFX3D()
   {
   }

   public final void setColor(BaseColorFX color)
   {
      this.color = color;
   }

   public final void setColor(Color color)
   {
      this.color = new SimpleColorFX(color);
   }

   public final BaseColorFX getColor()
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
