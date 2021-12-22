package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCapsuleFX3D;

public class YoCapsuleFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoCapsuleFX3D yoCapsuleFX3D = new YoCapsuleFX3D();
      yoCapsuleFX3D.setAxis(new Tuple3DProperty(-1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0)));
      yoCapsuleFX3D.setLength(0.3);
      yoCapsuleFX3D.setRadius(0.05);
      yoCapsuleFX3D.setColor(Color.AQUAMARINE);
      yoCapsuleFX3D.render();
      yoCapsuleFX3D.computeBackground();
      yoCapsuleFX3D.render();
      Simple3DViewer.view3DObjects(yoCapsuleFX3D.getNode());
   }
}
