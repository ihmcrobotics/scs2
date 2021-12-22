package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoEllipsoidFX3D;

public class YoEllipsoidFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoEllipsoidFX3D yoEllipsoidFX3D = new YoEllipsoidFX3D();
      Quaternion q = new Quaternion();
      q.appendYawRotation(-1.5);
      q.appendPitchRotation(-0.4);
      q.appendRollRotation(-0.2);
      yoEllipsoidFX3D.setOrientation(new QuaternionProperty(q.getX(), q.getY(), q.getZ(), q.getS()));
      yoEllipsoidFX3D.setRadii(new Tuple3DProperty(0.25, 0.4, 0.1));
      yoEllipsoidFX3D.setColor(Color.AQUAMARINE);
      yoEllipsoidFX3D.computeBackground();
      yoEllipsoidFX3D.render();
      Simple3DViewer.view3DObjects(yoEllipsoidFX3D.getNode());
   }
}
