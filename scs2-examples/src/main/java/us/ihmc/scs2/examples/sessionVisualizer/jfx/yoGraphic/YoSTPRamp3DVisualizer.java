package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPRampFX3D;


public class YoSTPRamp3DVisualizer
{
   public static void main(String[] args)
   {
      YoSTPRampFX3D YoRampFX3D = new YoSTPRampFX3D();
      Quaternion q = new Quaternion();
      q.appendPitchRotation(0.4);
      q.appendRollRotation(0.3);
      YoRampFX3D.setOrientation(new QuaternionProperty(q.getX(), q.getY(), q.getZ(), q.getS()));
      YoRampFX3D.setSize(new Tuple3DProperty(0.5, 0.25, 0.25));
      YoRampFX3D.setColor(Color.AQUAMARINE);
      YoRampFX3D.setMinimumMargin(0.015);
      YoRampFX3D.setMaximumMargin(0.035);
      YoRampFX3D.render();
      YoRampFX3D.computeBackground();
      YoRampFX3D.render();
      Simple3DViewer.view3DObjects(YoRampFX3D.getNode());
   }
}
