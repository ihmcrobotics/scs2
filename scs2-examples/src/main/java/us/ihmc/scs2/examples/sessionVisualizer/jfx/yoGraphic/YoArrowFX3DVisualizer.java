package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoArrowFX3D;

public class YoArrowFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoArrowFX3D yoArrowFX3D = new YoArrowFX3D();
      yoArrowFX3D.setOrigin(new Tuple3DProperty(0.0, 0.0, 0.0));
      yoArrowFX3D.setDirection(new Tuple3DProperty(-1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0)));
      yoArrowFX3D.setBodyLength(0.3);
      yoArrowFX3D.setBodyRadius(0.01);
      yoArrowFX3D.setHeadLength(0.06);
      yoArrowFX3D.setHeadRadius(0.025);
      yoArrowFX3D.setColor(Color.AQUAMARINE);
      yoArrowFX3D.computeBackground();
      yoArrowFX3D.render();
      Simple3DViewer.view3DObjects(yoArrowFX3D.getNode());
   }
}
