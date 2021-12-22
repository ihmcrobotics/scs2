package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple2DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoLineFX2D;

public class YoLineFX2DVisualizer
{
   public static void main(String[] args)
   {
      YoLineFX2D yoLineFX2D = new YoLineFX2D();
      yoLineFX2D.setOrigin(new Tuple2DProperty(-0.25, 0.25));
      yoLineFX2D.setDirection(null);
      yoLineFX2D.setDestination(new Tuple2DProperty(0.25, -0.25));
      yoLineFX2D.setStrokeWidth(3.0);
      yoLineFX2D.setStrokeColor(Color.AQUAMARINE.darker());
      Simple2DViewer.view2DObjects(() ->
      {
         yoLineFX2D.render();
         yoLineFX2D.computeBackground();
      }, yoLineFX2D.getNode());
   }
}
