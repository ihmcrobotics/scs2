package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class JavaFXToEuclidConversions
{
   public static Point2D convertPoint2D(javafx.geometry.Point2D jfxPoint2D)
   {
      return new Point2D(jfxPoint2D.getX(), jfxPoint2D.getY());
   }

   public static Vector2D convertVector2D(javafx.geometry.Point2D jfxPoint2D)
   {
      return new Vector2D(jfxPoint2D.getX(), jfxPoint2D.getY());
   }

   public static Point2D convertPoint2D(javafx.geometry.Point3D jfxPoint3D)
   {
      return new Point2D(jfxPoint3D.getX(), jfxPoint3D.getY());
   }

   public static Vector2D convertVector2D(javafx.geometry.Point3D jfxPoint3D)
   {
      return new Vector2D(jfxPoint3D.getX(), jfxPoint3D.getY());
   }

   public static Point3D convertPoint3D(javafx.geometry.Point3D jfxPoint3D)
   {
      return new Point3D(jfxPoint3D.getX(), jfxPoint3D.getY(), jfxPoint3D.getZ());
   }

   public static Vector3D convertVector3D(javafx.geometry.Point3D jfxPoint3D)
   {
      return new Vector3D(jfxPoint3D.getX(), jfxPoint3D.getY(), jfxPoint3D.getZ());
   }
}
