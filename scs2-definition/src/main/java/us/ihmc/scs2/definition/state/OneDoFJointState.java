package us.ihmc.scs2.definition.state;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateReadOnly;

@XmlType(propOrder = {"configuration", "velocity", "acceleration", "effort"})
public class OneDoFJointState extends JointStateBase implements OneDoFJointStateBasics
{
   private double configuration;
   private double velocity;
   private double acceleration;
   private double effort;

   private final DMatrixRMaj temp = new DMatrixRMaj(1, 1);

   public OneDoFJointState()
   {
      clear();
   }

   public OneDoFJointState(double q)
   {
      this();
      setConfiguration(q);
   }

   public OneDoFJointState(double q, double qd)
   {
      this();
      setConfiguration(q);
      setVelocity(qd);
   }

   public OneDoFJointState(double q, double qd, double tau)
   {
      this();
      setConfiguration(q);
      setVelocity(qd);
      setEffort(tau);
   }

   public OneDoFJointState(JointStateReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof OneDoFJointStateReadOnly)
      {
         OneDoFJointStateBasics.super.set((OneDoFJointStateReadOnly) jointStateReadOnly);
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

   @XmlAttribute
   @Override
   public void setConfiguration(double q)
   {
      configuration = q;
   }

   @XmlAttribute
   @Override
   public void setVelocity(double qd)
   {
      velocity = qd;
   }

   @XmlAttribute
   @Override
   public void setAcceleration(double qdd)
   {
      acceleration = qdd;
   }

   @XmlAttribute
   @Override
   public void setEffort(double tau)
   {
      effort = tau;
   }

   @Override
   public double getConfiguration()
   {
      return configuration;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   @Override
   public double getAcceleration()
   {
      return acceleration;
   }

   @Override
   public double getEffort()
   {
      return effort;
   }

   @Override
   public OneDoFJointState copy()
   {
      return new OneDoFJointState(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, configuration);
      bits = EuclidHashCodeTools.addToHashCode(bits, velocity);
      bits = EuclidHashCodeTools.addToHashCode(bits, acceleration);
      bits = EuclidHashCodeTools.addToHashCode(bits, effort);
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

      OneDoFJointState other = (OneDoFJointState) object;

      if (Double.doubleToLongBits(configuration) != Double.doubleToLongBits(other.configuration))
         return false;
      if (Double.doubleToLongBits(velocity) != Double.doubleToLongBits(other.velocity))
         return false;
      if (Double.doubleToLongBits(acceleration) != Double.doubleToLongBits(other.acceleration))
         return false;
      if (Double.doubleToLongBits(effort) != Double.doubleToLongBits(other.effort))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      String ret = "1-DoF joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", q: " + configuration;
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", qd: " + velocity;
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", qdd: " + acceleration;
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", tau: " + effort;
      return ret;
   }
}
