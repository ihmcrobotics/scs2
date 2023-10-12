package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPCylinderFX3D;
//import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoCylinderFX3D;

public class YoSTPCylinder3DVisualizer
{
   public static void main(String[] args)
   {
      YoSTPCylinderFX3D YoCylinderFX3D = new YoSTPCylinderFX3D();
//      YoCylinderFX3D.setPosition(new Tuple3DProperty(0,0,0));
      YoCylinderFX3D.setAxis(new Tuple3DProperty(-1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0)));
      YoCylinderFX3D.setLength(new SimpleDoubleProperty(0.3));
      YoCylinderFX3D.setRadius(new SimpleDoubleProperty(0.05));
      YoCylinderFX3D.setMinimumMargin(0.01);
      YoCylinderFX3D.setMaximumMargin(0.05);
      YoCylinderFX3D.setColor(Color.AQUAMARINE);
      YoCylinderFX3D.render();
      YoCylinderFX3D.computeBackground();
      YoCylinderFX3D.render();
      Simple3DViewer.view3DObjects(YoCylinderFX3D.getNode());
   }
}
