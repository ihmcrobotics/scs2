package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.paint.Color;
import javafx.scene.shape.DrawMode;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public abstract class YoGraphicFX3D extends YoGraphicFX
{
   public static final Color DEFAULT_COLOR = Color.BLUE;

   protected BaseColorFX color = new SimpleColorFX(DEFAULT_COLOR);
   protected Property<DrawMode> drawModeProperty = new SimpleObjectProperty<>(this, "drawMode", DrawMode.FILL);

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

   public final void setDrawMode(DrawMode drawMode)
   {
      drawModeProperty().setValue(drawMode);
   }

   public final DrawMode getDrawMode()
   {
      return drawModeProperty().getValue();
   }

   public final Property<DrawMode> drawModeProperty()
   {
      return drawModeProperty;
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
