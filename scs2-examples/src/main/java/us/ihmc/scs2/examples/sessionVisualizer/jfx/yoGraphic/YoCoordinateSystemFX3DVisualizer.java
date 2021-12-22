package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YawPitchRollProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCoordinateSystemFX3D;

public class YoCoordinateSystemFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoCoordinateSystemFX3D yoCoordinateSystemFX3D = new YoCoordinateSystemFX3D();
      yoCoordinateSystemFX3D.setOrientation(new YawPitchRollProperty(Math.PI, 0.0, 0.0));
      yoCoordinateSystemFX3D.setBodyLength(0.3);
      yoCoordinateSystemFX3D.setBodyRadius(0.0075);
      yoCoordinateSystemFX3D.setHeadLength(0.05);
      yoCoordinateSystemFX3D.setHeadRadius(0.025);
      yoCoordinateSystemFX3D.setColor(Color.AQUAMARINE);
      yoCoordinateSystemFX3D.render();
      yoCoordinateSystemFX3D.computeBackground();
      yoCoordinateSystemFX3D.render();
      Simple3DViewer.view3DObjects(yoCoordinateSystemFX3D.getNode());
   }
}
