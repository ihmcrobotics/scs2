package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCylinderFX3D;

public class YoCylinderFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoCylinderFX3D yoCylinderFX3D = new YoCylinderFX3D();
      yoCylinderFX3D.setAxis(new Tuple3DProperty(-1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0)));
      yoCylinderFX3D.setLength(0.3);
      yoCylinderFX3D.setRadius(0.05);
      yoCylinderFX3D.setColor(Color.AQUAMARINE);
      yoCylinderFX3D.render();
      yoCylinderFX3D.computeBackground();
      yoCylinderFX3D.render();
      Simple3DViewer.view3DObjects(yoCylinderFX3D.getNode());
   }
}
