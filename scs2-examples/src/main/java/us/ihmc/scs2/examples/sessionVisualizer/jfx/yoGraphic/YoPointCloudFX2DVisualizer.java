package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import java.util.Random;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple2DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointcloudFX2D;

public class YoPointCloudFX2DVisualizer
{
   public static void main(String[] args)
   {
      YoPointcloudFX2D yoPointcloudFX2D = new YoPointcloudFX2D();

      Random random = new Random(453443);
      Point2D start = new Point2D(-0.2, 0.4);
      Point2D end = new Point2D(0.4, -0.2);
      Point2D p = new Point2D();

      int numberOfPoints = 25;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double alpha = i / (numberOfPoints - 1.0);
         p.interpolate(start, end, alpha);
         p.add(EuclidCoreRandomTools.nextPoint2D(random, 0.15));
         yoPointcloudFX2D.addPoint(new Tuple2DProperty(p.getX(), p.getY()));
      }
      yoPointcloudFX2D.setSize(0.025);
      yoPointcloudFX2D.setStrokeColor(Color.AQUAMARINE.darker());
      Simple2DViewer.view2DObjects(() ->
      {
         yoPointcloudFX2D.render();
         yoPointcloudFX2D.computeBackground();
      }, yoPointcloudFX2D.getNode());
   }
}
