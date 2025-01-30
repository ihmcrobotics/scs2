package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import jakarta.xml.bind.annotation.XmlElement;

public class IMUSensorDefinition extends SensorDefinition
{
   private double accelerationNoiseMean;
   private double accelerationNoiseStandardDeviation;
   private double accelerationBiasMean;
   private double accelerationBiasStandardDeviation;

   private double angularVelocityNoiseMean;
   private double angularVelocityNoiseStandardDeviation;
   private double angularVelocityBiasMean;
   private double angularVelocityBiasStandardDeviation;

   public IMUSensorDefinition()
   {
   }

   public IMUSensorDefinition(String name)
   {
      super(name);
   }

   public IMUSensorDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public IMUSensorDefinition(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      super(name, transformToJoint);
   }

   public IMUSensorDefinition(IMUSensorDefinition other)
   {
      super(other);

      accelerationNoiseMean = other.accelerationNoiseMean;
      accelerationNoiseStandardDeviation = other.accelerationNoiseStandardDeviation;
      accelerationBiasMean = other.accelerationBiasMean;
      accelerationBiasStandardDeviation = other.accelerationBiasStandardDeviation;

      angularVelocityNoiseMean = other.angularVelocityNoiseMean;
      angularVelocityNoiseStandardDeviation = other.angularVelocityNoiseStandardDeviation;
      angularVelocityBiasMean = other.angularVelocityBiasMean;
      angularVelocityBiasStandardDeviation = other.angularVelocityBiasStandardDeviation;
   }

   public void setAccelerationNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      setAccelerationNoiseMean(noiseMean);
      setAccelerationNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAccelerationBiasParameters(double biasMean, double biasStandardDeviation)
   {
      setAccelerationBiasMean(biasMean);
      setAccelerationBiasStandardDeviation(biasStandardDeviation);
   }

   public void setAngularVelocityNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      setAngularVelocityNoiseMean(noiseMean);
      setAngularVelocityNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAngularVelocityBiasParameters(double biasMean, double biasStandardDeviation)
   {
      setAngularVelocityNoiseMean(biasMean);
      setAngularVelocityNoiseStandardDeviation(biasStandardDeviation);
   }

   public double getAccelerationNoiseMean()
   {
      return accelerationNoiseMean;
   }

   @XmlElement
   public void setAccelerationNoiseMean(double accelerationNoiseMean)
   {
      this.accelerationNoiseMean = accelerationNoiseMean;
   }

   public double getAccelerationNoiseStandardDeviation()
   {
      return accelerationNoiseStandardDeviation;
   }

   @XmlElement
   public void setAccelerationNoiseStandardDeviation(double accelerationNoiseStandardDeviation)
   {
      this.accelerationNoiseStandardDeviation = accelerationNoiseStandardDeviation;
   }

   public double getAccelerationBiasMean()
   {
      return accelerationBiasMean;
   }

   @XmlElement
   public void setAccelerationBiasMean(double accelerationBiasMean)
   {
      this.accelerationBiasMean = accelerationBiasMean;
   }

   public double getAccelerationBiasStandardDeviation()
   {
      return accelerationBiasStandardDeviation;
   }

   @XmlElement
   public void setAccelerationBiasStandardDeviation(double accelerationBiasStandardDeviation)
   {
      this.accelerationBiasStandardDeviation = accelerationBiasStandardDeviation;
   }

   public double getAngularVelocityNoiseMean()
   {
      return angularVelocityNoiseMean;
   }

   @XmlElement
   public void setAngularVelocityNoiseMean(double angularVelocityNoiseMean)
   {
      this.angularVelocityNoiseMean = angularVelocityNoiseMean;
   }

   public double getAngularVelocityNoiseStandardDeviation()
   {
      return angularVelocityNoiseStandardDeviation;
   }

   @XmlElement
   public void setAngularVelocityNoiseStandardDeviation(double angularVelocityNoiseStandardDeviation)
   {
      this.angularVelocityNoiseStandardDeviation = angularVelocityNoiseStandardDeviation;
   }

   public double getAngularVelocityBiasMean()
   {
      return angularVelocityBiasMean;
   }

   @XmlElement
   public void setAngularVelocityBiasMean(double angularVelocityBiasMean)
   {
      this.angularVelocityBiasMean = angularVelocityBiasMean;
   }

   public double getAngularVelocityBiasStandardDeviation()
   {
      return angularVelocityBiasStandardDeviation;
   }

   @XmlElement
   public void setAngularVelocityBiasStandardDeviation(double angularVelocityBiasStandardDeviation)
   {
      this.angularVelocityBiasStandardDeviation = angularVelocityBiasStandardDeviation;
   }

   @Override
   public IMUSensorDefinition copy()
   {
      return new IMUSensorDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, accelerationNoiseMean);
      bits = EuclidHashCodeTools.addToHashCode(bits, accelerationNoiseStandardDeviation);
      bits = EuclidHashCodeTools.addToHashCode(bits, accelerationBiasMean);
      bits = EuclidHashCodeTools.addToHashCode(bits, accelerationBiasStandardDeviation);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocityNoiseMean);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocityNoiseStandardDeviation);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocityBiasMean);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocityBiasStandardDeviation);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      IMUSensorDefinition other = (IMUSensorDefinition) object;

      if (!EuclidCoreTools.equals(accelerationNoiseMean, other.accelerationNoiseMean))
         return false;
      if (!EuclidCoreTools.equals(accelerationNoiseStandardDeviation, other.accelerationNoiseStandardDeviation))
         return false;
      if (!EuclidCoreTools.equals(accelerationBiasMean, other.accelerationBiasMean))
         return false;
      if (!EuclidCoreTools.equals(accelerationBiasStandardDeviation, other.accelerationBiasStandardDeviation))
         return false;
      if (!EuclidCoreTools.equals(angularVelocityNoiseMean, other.angularVelocityNoiseMean))
         return false;
      if (!EuclidCoreTools.equals(angularVelocityNoiseStandardDeviation, other.angularVelocityNoiseStandardDeviation))
         return false;
      if (!EuclidCoreTools.equals(angularVelocityBiasMean, other.angularVelocityBiasMean))
         return false;
      if (!EuclidCoreTools.equals(angularVelocityBiasStandardDeviation, other.angularVelocityBiasStandardDeviation))
         return false;

      return true;
   }
}
