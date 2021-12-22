package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.function.Supplier;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.paint.Color;

public abstract class YoGraphicFX2D extends YoGraphicFX
{
   public static final DoubleProperty DEFAULT_STROKE_WIDTH = new SimpleDoubleProperty(1.5);
   protected DoubleProperty strokeWidth = DEFAULT_STROKE_WIDTH;
   protected Supplier<Color> fillColor = null;
   protected Supplier<Color> strokeColor = () -> Color.BLUE;

   public YoGraphicFX2D()
   {
   }

   public final void setFillColor(Supplier<Color> fillColor)
   {
      this.fillColor = fillColor;
   }

   public final void setFillColor(Color fillColor)
   {
      this.fillColor = () -> fillColor;
   }

   public final void setStrokeColor(Supplier<Color> strokeColor)
   {
      this.strokeColor = strokeColor;
   }

   public final void setStrokeColor(Color strokeColor)
   {
      this.strokeColor = () -> strokeColor;
   }

   public final void setStrokeWidth(DoubleProperty strokeWidth)
   {
      this.strokeWidth = strokeWidth;
   }

   public final void setStrokeWidth(double strokeWidth)
   {
      this.strokeWidth = new SimpleDoubleProperty(strokeWidth);
   }

   public final Supplier<Color> getFillColor()
   {
      return fillColor;
   }

   public final Supplier<Color> getStrokeColor()
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
