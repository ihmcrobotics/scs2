package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPCapsuleFX3D;

public class YoSTPCapsule3DVisualizer
{
   public static void main(String[] args)
   {
      YoSTPCapsuleFX3D yoCapsuleFX3D = new YoSTPCapsuleFX3D();
      yoCapsuleFX3D.setCenter(new Tuple3DProperty(0,0,0));
      yoCapsuleFX3D.setAxis(new Tuple3DProperty(-1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0)));
      yoCapsuleFX3D.setLength(new SimpleDoubleProperty(0.3));
      yoCapsuleFX3D.setRadius(new SimpleDoubleProperty(0.05));
      yoCapsuleFX3D.setMinimumMargin(0.01);
      yoCapsuleFX3D.setMaximumMargin(0.05);
      yoCapsuleFX3D.setColor(Color.AQUAMARINE);
      yoCapsuleFX3D.render();
      yoCapsuleFX3D.computeBackground();
      yoCapsuleFX3D.render();
      Simple3DViewer.view3DObjects(yoCapsuleFX3D.getNode());

   }
}