package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoConeFX3D;

public class YoConeFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoConeFX3D yoConeFX3D = new YoConeFX3D();
      yoConeFX3D.setAxis(new Tuple3DProperty(1.0 / Math.sqrt(3.0)-0.75, 1.0 / Math.sqrt(3.0)+0.5, 1.0 / Math.sqrt(3.0)));
      yoConeFX3D.setHeight(0.3);
      yoConeFX3D.setRadius(0.15);
      yoConeFX3D.setColor(Color.AQUAMARINE);
      yoConeFX3D.render();
      yoConeFX3D.computeBackground();
      yoConeFX3D.render();
      Simple3DViewer.view3DObjects(yoConeFX3D.getNode());
   }
}
