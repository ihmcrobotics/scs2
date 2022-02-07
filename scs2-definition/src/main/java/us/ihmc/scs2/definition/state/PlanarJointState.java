package us.ihmc.scs2.definition.state;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Set;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.PlanarJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.PlanarJointStateReadOnly;

public class PlanarJointState implements PlanarJointStateBasics
{
   private final Set<JointStateType> availableStates = EnumSet.noneOf(JointStateType.class);

   private double pitch = Double.NaN;
   private double positionX = Double.NaN;
   private double positionZ = Double.NaN;

   private double pitchVelocity = Double.NaN;
   private double linearVelocityX = Double.NaN;
   private double linearVelocityZ = Double.NaN;

   private double pitchAcceleration = Double.NaN;
   private double linearAccelerationX = Double.NaN;
   private double linearAccelerationZ = Double.NaN;

   private double torqueY = Double.NaN;
   private double forceX = Double.NaN;
   private double forceZ = Double.NaN;

   private final DMatrixRMaj temp = new DMatrixRMaj(3, 1);

   public PlanarJointState()
   {
   }

   public PlanarJointState(JointStateReadOnly other)
   {
      set(other);
   }

   @Override
   public void clear()
   {
      availableStates.clear();
   }

   public void set(PlanarJointState other)
   {
      pitch = other.pitch;
      positionX = other.positionX;
      positionZ = other.positionZ;
      pitchVelocity = other.pitchVelocity;
      linearVelocityX = other.linearVelocityX;
      linearVelocityZ = other.linearVelocityZ;
      pitchAcceleration = other.pitchAcceleration;
      linearAccelerationX = other.linearAccelerationX;
      linearAccelerationZ = other.linearAccelerationZ;
      torqueY = other.torqueY;
      forceX = other.forceX;
      forceZ = other.forceZ;
      availableStates.addAll(other.availableStates);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof PlanarJointState)
      {
         set((PlanarJointState) jointStateReadOnly);
      }
      else if (jointStateReadOnly instanceof PlanarJointStateReadOnly)
      {
         PlanarJointStateBasics.super.set((PlanarJointStateReadOnly) jointStateReadOnly);
      }
      else
      {
         if (jointStateReadOnly.getConfigurationSize() != getConfigurationSize() || jointStateReadOnly.getDegreesOfFreedom() != getDegreesOfFreedom())
            throw new IllegalArgumentException("Dimension mismatch");
         clear();
         if (jointStateReadOnly.hasOutputFor(JointStateType.CONFIGURATION))
         {
            jointStateReadOnly.getConfiguration(0, temp);
            setConfiguration(0, temp);
         }
         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
      }
   }

   @Override
   public void setConfiguration(double pitch, double positionX, double positionZ)
   {
      this.pitch = pitch;
      this.positionX = positionX;
      this.positionZ = positionZ;
      availableStates.add(JointStateType.CONFIGURATION);
   }

   @Override
   public void setVelocity(double pitchVelocity, double linearVelocityX, double linearVelocityZ)
   {
      this.pitchVelocity = pitchVelocity;
      this.linearVelocityX = linearVelocityX;
      this.linearVelocityZ = linearVelocityZ;
      availableStates.add(JointStateType.VELOCITY);
   }

   @Override
   public void setAcceleration(double pitchAcceleration, double linearAccelerationX, double linearAccelerationZ)
   {
      this.pitchAcceleration = pitchAcceleration;
      this.linearAccelerationX = linearAccelerationX;
      this.linearAccelerationZ = linearAccelerationZ;
      availableStates.add(JointStateType.ACCELERATION);
   }

   @Override
   public void setEffort(double torqueY, double forceX, double forceZ)
   {
      this.torqueY = torqueY;
      this.forceX = forceX;
      this.forceZ = forceZ;
      availableStates.add(JointStateType.EFFORT);
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      return availableStates.contains(query);
   }

   @Override
   public double getPitch()
   {
      return pitch;
   }

   @Override
   public double getPositionX()
   {
      return positionX;
   }

   @Override
   public double getPositionZ()
   {
      return positionZ;
   }

   @Override
   public double getPitchVelocity()
   {
      return pitchVelocity;
   }

   @Override
   public double getLinearVelocityX()
   {
      return linearVelocityX;
   }

   @Override
   public double getLinearVelocityZ()
   {
      return linearVelocityZ;
   }

   @Override
   public double getPitchAcceleration()
   {
      return pitchAcceleration;
   }

   @Override
   public double getLinearAccelerationX()
   {
      return linearAccelerationX;
   }

   @Override
   public double getLinearAccelerationZ()
   {
      return linearAccelerationZ;
   }

   @Override
   public double getTorqueY()
   {
      return torqueY;
   }

   @Override
   public double getForceX()
   {
      return forceX;
   }

   @Override
   public double getForceZ()
   {
      return forceZ;
   }

   @Override
   public PlanarJointState copy()
   {
      return new PlanarJointState(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, availableStates);
      if (availableStates.contains(JointStateType.CONFIGURATION))
      {
         bits = EuclidHashCodeTools.addToHashCode(bits, pitch);
         bits = EuclidHashCodeTools.addToHashCode(bits, positionX);
         bits = EuclidHashCodeTools.addToHashCode(bits, positionZ);
      }
      if (availableStates.contains(JointStateType.VELOCITY))
      {
         bits = EuclidHashCodeTools.addToHashCode(bits, pitchVelocity);
         bits = EuclidHashCodeTools.addToHashCode(bits, linearVelocityX);
         bits = EuclidHashCodeTools.addToHashCode(bits, linearVelocityZ);
      }
      if (availableStates.contains(JointStateType.ACCELERATION))
      {
         bits = EuclidHashCodeTools.addToHashCode(bits, pitchAcceleration);
         bits = EuclidHashCodeTools.addToHashCode(bits, linearAccelerationX);
         bits = EuclidHashCodeTools.addToHashCode(bits, linearAccelerationZ);
      }
      if (availableStates.contains(JointStateType.EFFORT))
      {
         bits = EuclidHashCodeTools.addToHashCode(bits, torqueY);
         bits = EuclidHashCodeTools.addToHashCode(bits, forceX);
         bits = EuclidHashCodeTools.addToHashCode(bits, forceZ);
      }
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      PlanarJointState other = (PlanarJointState) object;

      if (!Objects.equals(availableStates, other.availableStates))
         return false;
      if (availableStates.contains(JointStateType.CONFIGURATION))
      {
         if (Double.doubleToLongBits(pitch) != Double.doubleToLongBits(other.pitch))
            return false;
         if (Double.doubleToLongBits(positionX) != Double.doubleToLongBits(other.positionX))
            return false;
         if (Double.doubleToLongBits(positionZ) != Double.doubleToLongBits(other.positionZ))
            return false;
      }
      if (availableStates.contains(JointStateType.VELOCITY))
      {
         if (Double.doubleToLongBits(pitchVelocity) != Double.doubleToLongBits(other.pitchVelocity))
            return false;
         if (Double.doubleToLongBits(linearVelocityX) != Double.doubleToLongBits(other.linearVelocityX))
            return false;
         if (Double.doubleToLongBits(linearVelocityZ) != Double.doubleToLongBits(other.linearVelocityZ))
            return false;
      }
      if (availableStates.contains(JointStateType.ACCELERATION))
      {
         if (Double.doubleToLongBits(pitchAcceleration) != Double.doubleToLongBits(other.pitchAcceleration))
            return false;
         if (Double.doubleToLongBits(linearAccelerationX) != Double.doubleToLongBits(other.linearAccelerationX))
            return false;
         if (Double.doubleToLongBits(linearAccelerationZ) != Double.doubleToLongBits(other.linearAccelerationZ))
            return false;
      }
      if (availableStates.contains(JointStateType.EFFORT))
      {
         if (Double.doubleToLongBits(torqueY) != Double.doubleToLongBits(other.torqueY))
            return false;
         if (Double.doubleToLongBits(forceX) != Double.doubleToLongBits(other.forceX))
            return false;
         if (Double.doubleToLongBits(forceZ) != Double.doubleToLongBits(other.forceZ))
            return false;
      }

      return true;
   }
}
