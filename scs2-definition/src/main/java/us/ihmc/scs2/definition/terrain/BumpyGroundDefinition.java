package us.ihmc.scs2.definition.terrain;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.visual.HeightMapDefinition;

public class BumpyGroundDefinition extends HeightMapGroundDefinition
{
   private static final double xMinDefault = -10.0, xMaxDefault = 10.0, yMinDefault = -10.0, yMaxDefault = 10.0;
   private static final double xAmp1Default = 0.2, xFreq1Default = 0.1, xAmp2Default = 0.1, xFreq2Default = 0.5;
   private static final double yAmp1Default = 0.1, yFreq1Default = 0.07, yAmp2Default = 0.05, yFreq2Default = 0.37;

   public BumpyGroundDefinition()
   {
      this(xAmp1Default, xFreq1Default, xAmp2Default, xFreq2Default, yAmp1Default, yFreq1Default, yAmp2Default, yFreq2Default);
   }

   public BumpyGroundDefinition(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2,
                             double flatGroundBoxWidthAtZero)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault, flatGroundBoxWidthAtZero);
   }

   public BumpyGroundDefinition(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }

   public BumpyGroundDefinition(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2, double xMin,
                             double xMax, double yMin, double yMax)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, xMin, xMax, yMin, yMax, 0.0);
   }

   public BumpyGroundDefinition(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2, double xMin,
                             double xMax, double yMin, double yMax, double flatGroundBoxWidthAtZero)
   {
      super(new BumpyDefinition(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, xMin, xMax, yMin, yMax, flatGroundBoxWidthAtZero));
   }

   private static class BumpyDefinition implements HeightMapDefinition
   {
      private final BoundingBox2D boundingBox;

      private final double xAmp1, xFreq1, xAmp2, xFreq2;
      private final double yAmp1, yFreq1, yAmp2, yFreq2;

      private final double flatGroundBoxWidthAtZero;

      public BumpyDefinition(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2, double xMin,
                                   double xMax, double yMin, double yMax, double flatGroundBoxWidthAtZero)
      {
         this.xAmp1 = xAmp1;
         this.xFreq1 = xFreq1;
         this.xAmp2 = xAmp2;
         this.xFreq2 = xFreq2;

         this.yAmp1 = yAmp1;
         this.yFreq1 = yFreq1;
         this.yAmp2 = yAmp2;
         this.yFreq2 = yFreq2;

         this.flatGroundBoxWidthAtZero = flatGroundBoxWidthAtZero;

         boundingBox = new BoundingBox2D(xMin, yMin, xMax, yMax);
      }

      @Override
      public BoundingBox2DReadOnly getBoundingBox()
      {
         return boundingBox;
      }

      @Override
      public double getHeightAt(double x, double y)
      {
         if (Math.abs(x) < flatGroundBoxWidthAtZero / 2.0 && Math.abs(y) < flatGroundBoxWidthAtZero / 2.0)
         {
            return 0.0;
         }

         double height =
               (xAmp1 * Math.sin(2.0 * Math.PI * xFreq1 * x) + xAmp2 * Math.sin(2.0 * Math.PI * xFreq2 * x)) + (yAmp1 * Math.sin(2.0 * Math.PI * yFreq1 * y)
                                                                                                                + yAmp2 * Math.sin(2.0 * Math.PI * yFreq2 * y));

         return height;
      }

      @Override
      public Vector3DReadOnly getNormalAt(double x, double y)
      {
         if (Math.abs(x) < flatGroundBoxWidthAtZero / 2.0 && Math.abs(y) < flatGroundBoxWidthAtZero / 2.0)
         {
            return new Vector3D(0.0, 0.0, 1.0);
         }

         Vector3D normal = new Vector3D();
         double dzdx =
               xAmp1 * 2.0 * Math.PI * xFreq1 * Math.cos(2.0 * Math.PI * xFreq1 * x) + xAmp2 * 2.0 * Math.PI * xFreq2 * Math.cos(2.0 * Math.PI * xFreq2 * x);
         double dzdy =
               yAmp1 * 2.0 * Math.PI * yFreq1 * Math.cos(2.0 * Math.PI * yFreq1 * y) + yAmp2 * 2.0 * Math.PI * yFreq2 * Math.cos(2.0 * Math.PI * yFreq2 * y);
         normal.setX(-dzdx);
         normal.setY(-dzdy);
         normal.setZ(1.0);

         normal.normalize();

         return normal;
      }

   }
}
