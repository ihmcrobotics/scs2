package us.ihmc.scs2.definition.terrain;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.visual.HeightMapDefinition;

public class RollingGroundDefinition extends HeightMapGroundDefinition
{
   private static final double xMinDefault = -20.0, xMaxDefault = 20.0, yMinDefault = -20.0, yMaxDefault = 20.0;
   private static final double amplitudeDefault = 0.1, frequencyDefault = 0.3, offsetDefault = 0.0;

   public RollingGroundDefinition()
   {
      this(amplitudeDefault, frequencyDefault, offsetDefault);
   }

   public RollingGroundDefinition(double amplitude, double frequency, double offset)
   {
      this(amplitude, frequency, offset, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }

   public RollingGroundDefinition(double amplitude, double frequency, double offset, double xMin, double xMax, double yMin, double yMax)
   {
      super(new RollingDefinition(amplitude, frequency, offset, xMin, xMax, yMin, yMax));
   }

   private static class RollingDefinition implements HeightMapDefinition
   {
      private final BoundingBox2D boundingBox;

      protected final double amplitude, frequency, offset;

      public RollingDefinition(double amplitude, double frequency, double offset, double xMin, double xMax, double yMin, double yMax)
      {
         this.amplitude = amplitude;
         this.frequency = frequency;
         this.offset = offset;

         boundingBox = new BoundingBox2D(new Point2D(xMin, yMin), new Point2D(xMax, yMax));
      }

      @Override
      public BoundingBox2DReadOnly getBoundingBox()
      {
         return boundingBox;
      }

      @Override
      public double getHeightAt(double x, double y)
      {
         return amplitude * Math.sin(2.0 * Math.PI * frequency * (x + offset));
      }

      @Override
      public Vector3DReadOnly getNormalAt(double x, double y)
      {
         double dzdx = amplitude * 2.0 * Math.PI * frequency * Math.cos(2.0 * Math.PI * frequency * (x + offset));

         Vector3D normal = new Vector3D();
         normal.setX(-dzdx);
         normal.setY(0.0);
         normal.setZ(1.0);

         normal.normalize();

         return normal;
      }
   }
}
