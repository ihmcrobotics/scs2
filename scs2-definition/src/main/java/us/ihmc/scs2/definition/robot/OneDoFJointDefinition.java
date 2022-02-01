package us.ihmc.scs2.definition.robot;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public abstract class OneDoFJointDefinition extends JointDefinition
{
   private Vector3D axis = new Vector3D();

   private double positionLowerLimit = Double.NEGATIVE_INFINITY, positionUpperLimit = Double.POSITIVE_INFINITY;
   private double velocityLowerLimit = Double.NEGATIVE_INFINITY, velocityUpperLimit = Double.POSITIVE_INFINITY;
   private double effortLowerLimit = Double.NEGATIVE_INFINITY, effortUpperLimit = Double.POSITIVE_INFINITY;
   private double damping = -1.0;
   private double stiction = -1.0;

   private double kpSoftLimitStop = -1.0;
   private double kdSoftLimitStop = -1.0;
   private double dampingVelocitySoftLimit = -1.0;

   private OneDoFJointState initialJointState;

   public OneDoFJointDefinition()
   {
   }

   public OneDoFJointDefinition(String name)
   {
      super(name);
   }

   public OneDoFJointDefinition(String name, Tuple3DReadOnly offsetFromParent, Vector3DReadOnly axis)
   {
      super(name, offsetFromParent);
      setAxis(axis);
   }

   public OneDoFJointDefinition(OneDoFJointDefinition other)
   {
      super(other);

      axis.set(other.axis);
      positionLowerLimit = other.positionLowerLimit;
      positionUpperLimit = other.positionUpperLimit;
      velocityLowerLimit = other.velocityLowerLimit;
      velocityUpperLimit = other.velocityUpperLimit;
      effortLowerLimit = other.effortLowerLimit;
      effortUpperLimit = other.effortUpperLimit;
      damping = other.damping;
      stiction = other.stiction;
      kpSoftLimitStop = other.kpSoftLimitStop;
      kdSoftLimitStop = other.kdSoftLimitStop;
      dampingVelocitySoftLimit = other.dampingVelocitySoftLimit;
      initialJointState = other.initialJointState == null ? null : other.initialJointState.copy();
   }

   @XmlElement
   public void setAxis(Vector3D axis)
   {
      this.axis = axis;
   }

   public void setAxis(Vector3DReadOnly axis)
   {
      this.axis.set(axis);
   }

   public Vector3D getAxis()
   {
      return axis;
   }

   @XmlElement
   public void setPositionLowerLimit(double positionLowerLimit)
   {
      this.positionLowerLimit = positionLowerLimit;
   }

   @XmlElement
   public void setPositionUpperLimit(double positionUpperLimit)
   {
      this.positionUpperLimit = positionUpperLimit;
   }

   public void setPositionLimits(double lowerLimit, double upperLimit)
   {
      setPositionLowerLimit(lowerLimit);
      setPositionUpperLimit(upperLimit);
   }

   public double getPositionLowerLimit()
   {
      return positionLowerLimit;
   }

   public double getPositionUpperLimit()
   {
      return positionUpperLimit;
   }

   @XmlElement
   public void setVelocityLowerLimit(double velocityLowerLimit)
   {
      this.velocityLowerLimit = velocityLowerLimit;
   }

   @XmlElement
   public void setVelocityUpperLimit(double velocityUpperLimit)
   {
      this.velocityUpperLimit = velocityUpperLimit;
   }

   public void setVelocityLimits(double lowerLimit, double upperLimit)
   {
      setVelocityLowerLimit(lowerLimit);
      setVelocityUpperLimit(upperLimit);
   }

   public void setVelocityLimits(double limit)
   {
      setVelocityLowerLimit(-limit);
      setVelocityUpperLimit(limit);
   }

   public double getVelocityLowerLimit()
   {
      return velocityLowerLimit;
   }

   public double getVelocityUpperLimit()
   {
      return velocityUpperLimit;
   }

   @XmlElement
   public void setEffortLowerLimit(double effortLowerLimit)
   {
      this.effortLowerLimit = effortLowerLimit;
   }

   @XmlElement
   public void setEffortUpperLimit(double effortUpperLimit)
   {
      this.effortUpperLimit = effortUpperLimit;
   }

   public void setEffortLimits(double lowerLimit, double upperLimit)
   {
      setEffortLowerLimit(lowerLimit);
      setEffortUpperLimit(upperLimit);
   }

   public void setEffortLimits(double limit)
   {
      setEffortLowerLimit(-limit);
      setEffortUpperLimit(limit);
   }

   public double getEffortLowerLimit()
   {
      return effortLowerLimit;
   }

   public double getEffortUpperLimit()
   {
      return effortUpperLimit;
   }

   @XmlElement
   public void setStiction(double stiction)
   {
      this.stiction = stiction;
   }

   public double getStiction()
   {
      return stiction;
   }

   @XmlElement
   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public double getDamping()
   {
      return damping;
   }

   @XmlElement
   public void setKpSoftLimitStop(double kpSoftLimitStop)
   {
      this.kpSoftLimitStop = kpSoftLimitStop;
   }

   @XmlElement
   public void setKdSoftLimitStop(double kdSoftLimitStop)
   {
      this.kdSoftLimitStop = kdSoftLimitStop;
   }

   public void setGainsSoftLimitStop(double kpSoftLimitStop, double kdSoftLimitStop)
   {
      this.kpSoftLimitStop = kpSoftLimitStop;
      this.kdSoftLimitStop = kdSoftLimitStop;
   }

   public double getKpSoftLimitStop()
   {
      return kpSoftLimitStop;
   }

   public double getKdSoftLimitStop()
   {
      return kdSoftLimitStop;
   }

   @XmlElement
   public void setDampingVelocitySoftLimit(double dampingVelocitySoftLimit)
   {
      this.dampingVelocitySoftLimit = dampingVelocitySoftLimit;
   }

   public double getDampingVelocitySoftLimit()
   {
      return dampingVelocitySoftLimit;
   }

   public void setInitialJointState(double q)
   {
      if (initialJointState == null)
         setInitialJointState(new OneDoFJointState());
      initialJointState.setConfiguration(q);
   }

   public void setInitialJointState(double q, double qd)
   {
      if (initialJointState == null)
         setInitialJointState(new OneDoFJointState());
      initialJointState.setConfiguration(q);
      initialJointState.setVelocity(qd);
   }

   public void setInitialJointState(OneDoFJointState initialJointState)
   {
      this.initialJointState = initialJointState;
   }

   @Override
   public void setInitialJointState(JointStateReadOnly initialJointState)
   {
      if (initialJointState instanceof OneDoFJointState)
         setInitialJointState((OneDoFJointState) initialJointState);
      else if (this.initialJointState == null)
         this.initialJointState = new OneDoFJointState(initialJointState);
      else
         this.initialJointState.set(initialJointState);
   }

   @Override
   public OneDoFJointState getInitialJointState()
   {
      return initialJointState;
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, axis);
      bits = EuclidHashCodeTools.addToHashCode(bits, positionLowerLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, positionUpperLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, velocityLowerLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, velocityUpperLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, effortLowerLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, effortUpperLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, damping);
      bits = EuclidHashCodeTools.addToHashCode(bits, stiction);
      bits = EuclidHashCodeTools.addToHashCode(bits, kpSoftLimitStop);
      bits = EuclidHashCodeTools.addToHashCode(bits, kdSoftLimitStop);
      bits = EuclidHashCodeTools.addToHashCode(bits, dampingVelocitySoftLimit);
      bits = EuclidHashCodeTools.addToHashCode(bits, initialJointState);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      OneDoFJointDefinition other = (OneDoFJointDefinition) object;

      if (!Objects.equals(axis, other.axis))
         return false;
      if (Double.doubleToLongBits(positionLowerLimit) != Double.doubleToLongBits(other.positionLowerLimit))
         return false;
      if (Double.doubleToLongBits(positionUpperLimit) != Double.doubleToLongBits(other.positionUpperLimit))
         return false;
      if (Double.doubleToLongBits(velocityLowerLimit) != Double.doubleToLongBits(other.velocityLowerLimit))
         return false;
      if (Double.doubleToLongBits(velocityUpperLimit) != Double.doubleToLongBits(other.velocityUpperLimit))
         return false;
      if (Double.doubleToLongBits(effortLowerLimit) != Double.doubleToLongBits(other.effortLowerLimit))
         return false;
      if (Double.doubleToLongBits(effortUpperLimit) != Double.doubleToLongBits(other.effortUpperLimit))
         return false;
      if (Double.doubleToLongBits(damping) != Double.doubleToLongBits(other.damping))
         return false;
      if (Double.doubleToLongBits(stiction) != Double.doubleToLongBits(other.stiction))
         return false;
      if (Double.doubleToLongBits(kpSoftLimitStop) != Double.doubleToLongBits(other.kpSoftLimitStop))
         return false;
      if (Double.doubleToLongBits(kdSoftLimitStop) != Double.doubleToLongBits(other.kdSoftLimitStop))
         return false;
      if (Double.doubleToLongBits(dampingVelocitySoftLimit) != Double.doubleToLongBits(other.dampingVelocitySoftLimit))
         return false;
      if (!Objects.equals(initialJointState, other.initialJointState))
         return false;

      return true;
   }
}
