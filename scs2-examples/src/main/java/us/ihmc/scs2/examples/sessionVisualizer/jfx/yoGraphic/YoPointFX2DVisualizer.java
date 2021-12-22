package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple2DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointFX2D;

public class YoPointFX2DVisualizer
{
   public static void main(String[] args)
   {
      YoPointFX2D yoPointFX2D = new YoPointFX2D();
      yoPointFX2D.setSize(0.05);
      yoPointFX2D.setStrokeWidth(1.5);
      yoPointFX2D.setStrokeColor(Color.AQUAMARINE.darker());
      Simple2DViewer.view2DObjects(() ->
      {
         yoPointFX2D.render();
         yoPointFX2D.computeBackground();
      }, yoPointFX2D.getNode());
   }
}
