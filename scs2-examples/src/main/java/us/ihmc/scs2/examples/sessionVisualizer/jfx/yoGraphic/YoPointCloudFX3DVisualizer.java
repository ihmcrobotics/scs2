package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import java.util.Random;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointcloudFX3D;

public class YoPointCloudFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoPointcloudFX3D yoPointcloudFX3D = new YoPointcloudFX3D();

      Random random = new Random(453443);
      Point3D start = new Point3D(-0.2, 0.4, -0.6);
      Point3D end = new Point3D(0.4, -0.2, 0.2);
      Point3D p = new Point3D();

      int numberOfPoints = 25;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double alpha = i / (numberOfPoints - 1.0);
         p.interpolate(start, end, alpha);
         p.add(EuclidCoreRandomTools.nextPoint3D(random, 0.15));
         yoPointcloudFX3D.addPoint(new Tuple3DProperty(p.getX(), p.getY(), p.getZ()));
      }
      yoPointcloudFX3D.setSize(0.025);
      yoPointcloudFX3D.setColor(Color.AQUAMARINE);
      yoPointcloudFX3D.render();
      yoPointcloudFX3D.computeBackground();
      yoPointcloudFX3D.render();
      Simple3DViewer.view3DObjects(yoPointcloudFX3D.getNode());
   }
}
