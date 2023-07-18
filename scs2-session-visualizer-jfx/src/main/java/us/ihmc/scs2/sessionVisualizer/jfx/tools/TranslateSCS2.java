package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.scene.transform.Translate;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class TranslateSCS2 extends Translate implements Tuple3DBasics
{

   public TranslateSCS2()
   {
   }

   public TranslateSCS2(double x, double y, double z)
   {
      super(x, y, z);
   }

   public TranslateSCS2(double x, double y)
   {
      super(x, y);
   }

   public void set(javafx.geometry.Point3D point3D)
   {
      set(point3D.getX(), point3D.getY(), point3D.getZ());
   }

   public void setFrom(javafx.scene.transform.Transform transform)
   {
      setX(transform.getTx());
      setY(transform.getTy());
      setZ(transform.getTz());
   }

   @Override
   public boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof TranslateSCS2))
         return false;
      TranslateSCS2 other = (TranslateSCS2) geometry;
      return differenceNorm(other) <= epsilon;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      Vector3D temp = new Vector3D(this);
      transform.transform(temp);
      set(temp);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      Vector3D temp = new Vector3D(this);
      transform.inverseTransform(temp);
      set(temp);
   }
}
