package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoRampFX3D;

public class YoRampFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoRampFX3D yoRampFX3D = new YoRampFX3D();
      Quaternion q = new Quaternion();
      q.appendPitchRotation(0.4);
      q.appendRollRotation(-0.3);
      yoRampFX3D.setOrientation(new QuaternionProperty(q.getX(), q.getY(), q.getZ(), q.getS()));
      yoRampFX3D.setSize(new Tuple3DProperty(0.5, 0.25, 0.25));
      yoRampFX3D.setColor(Color.AQUAMARINE);
      yoRampFX3D.computeBackground();
      yoRampFX3D.render();
      Simple3DViewer.view3DObjects(yoRampFX3D.getNode());
   }
}
