package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPBoxFX3D;
//import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoBoxFX3D;

public class YoSTPBox3DVisualizer
{
   public static void main(String[] args)
   {
      YoSTPBoxFX3D yoBoxFX3D = new YoSTPBoxFX3D();
      Quaternion q = new Quaternion();
      q.appendPitchRotation(0.4);
      q.appendRollRotation(0.3);
      yoBoxFX3D.setOrientation(new QuaternionProperty(q.getX(), q.getY(), q.getZ(), q.getS()));
      yoBoxFX3D.setSize(new Tuple3DProperty(0.25, 0.25, 0.25));
      yoBoxFX3D.setColor(Color.AQUAMARINE);
      yoBoxFX3D.setMinimumMargin(0.015);
      yoBoxFX3D.setMaximumMargin(0.035);
      yoBoxFX3D.render();
      yoBoxFX3D.computeBackground();
      yoBoxFX3D.render();
      Simple3DViewer.view3DObjects(yoBoxFX3D.getNode());
   }
}
