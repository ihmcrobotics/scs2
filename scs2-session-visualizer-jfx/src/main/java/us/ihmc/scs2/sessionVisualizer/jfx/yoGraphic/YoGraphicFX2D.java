package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.paint.Color;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public abstract class YoGraphicFX2D extends YoGraphicFX
{
   public static final DoubleProperty DEFAULT_STROKE_WIDTH = new SimpleDoubleProperty(1.5);
   protected DoubleProperty strokeWidth = DEFAULT_STROKE_WIDTH;
   protected BaseColorFX fillColor = null;
   protected BaseColorFX strokeColor = new SimpleColorFX(Color.BLUE);

   public YoGraphicFX2D()
   {
   }

   public final void setFillColor(BaseColorFX fillColor)
   {
      this.fillColor = fillColor;
   }

   public final void setFillColor(Color fillColor)
   {
      this.fillColor = new SimpleColorFX(fillColor);
   }

   public final void setStrokeColor(BaseColorFX strokeColor)
   {
      this.strokeColor = strokeColor;
   }

   public final void setStrokeColor(Color strokeColor)
   {
      this.strokeColor = new SimpleColorFX(strokeColor);
   }

   public final void setStrokeWidth(DoubleProperty strokeWidth)
   {
      this.strokeWidth = strokeWidth;
   }

   public final void setStrokeWidth(double strokeWidth)
   {
      this.strokeWidth = new SimpleDoubleProperty(strokeWidth);
   }

   public final BaseColorFX getFillColor()
   {
      return fillColor;
   }

   public final BaseColorFX getStrokeColor()
   {
      return strokeColor;
   }

   public final DoubleProperty getStrokeWidth()
   {
      return strokeWidth;
   }

   @Override
   public void detachFromParent()
   {
      YoGroupFX parentGroup = getParentGroup();

      if (parentGroup != null)
      {
         parentGroup.removeYoGraphicFX2D(this);
         parentGroupProperty().set(null);
      }
   }
}
