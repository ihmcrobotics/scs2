package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPointFX3D;

public class YoPointFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoPointFX3D yoPointFX3D = new YoPointFX3D();
      yoPointFX3D.setColor(Color.AQUAMARINE);
      yoPointFX3D.render();
      Simple3DViewer.view3DObjects(yoPointFX3D.getNode());
   }
}
