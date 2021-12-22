package us.ihmc.scs2.definition.robot;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

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
}
