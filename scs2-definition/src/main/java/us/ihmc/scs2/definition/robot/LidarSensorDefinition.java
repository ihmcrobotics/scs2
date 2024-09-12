package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import jakarta.xml.bind.annotation.XmlElement;

public class LidarSensorDefinition extends SensorDefinition
{
   private double sweepYawMin;
   private double sweepYawMax;

   private double heightPitchMin;
   private double heightPitchMax;

   private double minRange;
   private double maxRange;
   private double rangeResolution;

   private int pointsPerSweep;
   private int scanHeight;

   private double gaussianNoiseMean;
   private double gaussianNoiseStandardDeviation;

   public LidarSensorDefinition()
   {
   }

   public LidarSensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      super(name, transformToJoint);
   }

   public LidarSensorDefinition(LidarSensorDefinition other)
   {
      super(other);

      sweepYawMin = other.sweepYawMin;
      sweepYawMax = other.sweepYawMax;
      heightPitchMin = other.heightPitchMin;
      heightPitchMax = other.heightPitchMax;
      minRange = other.minRange;
      maxRange = other.maxRange;
      rangeResolution = other.rangeResolution;
      pointsPerSweep = other.pointsPerSweep;
      scanHeight = other.scanHeight;
      gaussianNoiseMean = other.gaussianNoiseMean;
      gaussianNoiseStandardDeviation = other.gaussianNoiseStandardDeviation;
   }

   public double getSweepYawMin()
   {
      return sweepYawMin;
   }

   public double getSweepYawMax()
   {
      return sweepYawMax;
   }

   public double getHeightPitchMin()
   {
      return heightPitchMin;
   }

   public double getHeightPitchMax()
   {
      return heightPitchMax;
   }

   public double getMinRange()
   {
      return minRange;
   }

   public double getMaxRange()
   {
      return maxRange;
   }

   public double getRangeResolution()
   {
      return rangeResolution;
   }

   public int getPointsPerSweep()
   {
      return pointsPerSweep;
   }

   public int getScanHeight()
   {
      return scanHeight;
   }

   public double getGaussianNoiseMean()
   {
      return gaussianNoiseMean;
   }

   public double getGaussianNoiseStandardDeviation()
   {
      return gaussianNoiseStandardDeviation;
   }

   public void setSweepYawLimits(double min, double max)
   {
      sweepYawMin = min;
      sweepYawMax = max;
   }

   @XmlElement
   public void setSweepYawMin(double sweepYawMin)
   {
      this.sweepYawMin = sweepYawMin;
   }

   @XmlElement
   public void setSweepYawMax(double sweepYawMax)
   {
      this.sweepYawMax = sweepYawMax;
   }

   public void setHeightPitchLimits(double min, double max)
   {
      heightPitchMin = min;
      heightPitchMax = max;
   }

   @XmlElement
   public void setHeightPitchMin(double heightPitchMin)
   {
      this.heightPitchMin = heightPitchMin;
   }

   @XmlElement
   public void setHeightPitchMax(double heightPitchMax)
   {
      this.heightPitchMax = heightPitchMax;
   }

   public void setRangeLimits(double min, double max)
   {
      minRange = min;
      maxRange = max;
   }

   @XmlElement
   public void setMinRange(double minRange)
   {
      this.minRange = minRange;
   }

   @XmlElement
   public void setMaxRange(double maxRange)
   {
      this.maxRange = maxRange;
   }

   @XmlElement
   public void setRangeResolution(double rangeResolution)
   {
      this.rangeResolution = rangeResolution;
   }

   @XmlElement
   public void setPointsPerSweep(int pointsPerSweep)
   {
      this.pointsPerSweep = pointsPerSweep;
   }

   @XmlElement
   public void setScanHeight(int scanHeight)
   {
      this.scanHeight = scanHeight;
   }

   @XmlElement
   public void setGaussianNoiseMean(double gaussianNoiseMean)
   {
      this.gaussianNoiseMean = gaussianNoiseMean;
   }

   @XmlElement
   public void setGaussianNoiseStandardDeviation(double gaussianNoiseStandardDeviation)
   {
      this.gaussianNoiseStandardDeviation = gaussianNoiseStandardDeviation;
   }

   @Override
   public LidarSensorDefinition copy()
   {
      return new LidarSensorDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, sweepYawMin);
      bits = EuclidHashCodeTools.addToHashCode(bits, sweepYawMax);
      bits = EuclidHashCodeTools.addToHashCode(bits, heightPitchMin);
      bits = EuclidHashCodeTools.addToHashCode(bits, heightPitchMax);
      bits = EuclidHashCodeTools.addToHashCode(bits, minRange);
      bits = EuclidHashCodeTools.addToHashCode(bits, maxRange);
      bits = EuclidHashCodeTools.addToHashCode(bits, rangeResolution);
      bits = EuclidHashCodeTools.addToHashCode(bits, pointsPerSweep);
      bits = EuclidHashCodeTools.addToHashCode(bits, scanHeight);
      bits = EuclidHashCodeTools.addToHashCode(bits, gaussianNoiseMean);
      bits = EuclidHashCodeTools.addToHashCode(bits, gaussianNoiseStandardDeviation);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      LidarSensorDefinition other = (LidarSensorDefinition) object;

      if (!EuclidCoreTools.equals(sweepYawMin, other.sweepYawMin))
         return false;
      if (!EuclidCoreTools.equals(sweepYawMax, other.sweepYawMax))
         return false;
      if (!EuclidCoreTools.equals(heightPitchMin, other.heightPitchMin))
         return false;
      if (!EuclidCoreTools.equals(heightPitchMax, other.heightPitchMax))
         return false;
      if (!EuclidCoreTools.equals(minRange, other.minRange))
         return false;
      if (!EuclidCoreTools.equals(maxRange, other.maxRange))
         return false;
      if (!EuclidCoreTools.equals(rangeResolution, other.rangeResolution))
         return false;
      if (pointsPerSweep != other.pointsPerSweep)
         return false;
      if (scanHeight != other.scanHeight)
         return false;
      if (!EuclidCoreTools.equals(gaussianNoiseMean, other.gaussianNoiseMean))
         return false;
      if (!EuclidCoreTools.equals(gaussianNoiseStandardDeviation, other.gaussianNoiseStandardDeviation))
         return false;

      return true;
   }
}
