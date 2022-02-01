package us.ihmc.scs2.definition.state;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateReadOnly;

public class SixDoFJointState extends JointStateBase implements SixDoFJointStateBasics
{
   private final Pose3D configuration = new Pose3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D angularAcceleration = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D torque = new Vector3D();
   private final Vector3D force = new Vector3D();

   private final DMatrixRMaj temp = new DMatrixRMaj(7, 1);

   public SixDoFJointState()
   {
      clear();
   }

   public SixDoFJointState(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      this();
      setConfiguration(orientation, position);
   }

   public SixDoFJointState(JointStateReadOnly other)
   {
      this();
      set(other);
   }

   @Override
   public void clear()
   {
      configuration.setToNaN();
      angularVelocity.setToNaN();
      linearVelocity.setToNaN();
      angularAcceleration.setToNaN();
      linearAcceleration.setToNaN();
      torque.setToNaN();
      force.setToNaN();
   }

   public void set(SixDoFJointState other)
   {
      configuration.set(other.configuration);
      angularVelocity.set(other.angularVelocity);
      linearVelocity.set(other.linearVelocity);
      angularAcceleration.set(other.angularAcceleration);
      linearAcceleration.set(other.linearAcceleration);
      torque.set(other.torque);
      force.set(other.force);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof SixDoFJointState)
      {
         set((SixDoFJointState) jointStateReadOnly);
      }
      else if (jointStateReadOnly instanceof SixDoFJointStateReadOnly)
      {
         SixDoFJointStateBasics.super.set((SixDoFJointStateReadOnly) jointStateReadOnly);
      }
      else
      {
         if (jointStateReadOnly.getConfigurationSize() != getConfigurationSize() || jointStateReadOnly.getDegreesOfFreedom() != getDegreesOfFreedom())
            throw new IllegalArgumentException("Dimension mismatch");

         if (jointStateReadOnly.hasOutputFor(JointStateType.CONFIGURATION))
         {
            jointStateReadOnly.getConfiguration(0, temp);
            setConfiguration(0, temp);
         }
         else
         {
            configuration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         else
         {
            angularVelocity.setToNaN();
            linearVelocity.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         else
         {
            angularAcceleration.setToNaN();
            linearAcceleration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
         else
         {
            torque.setToNaN();
            force.setToNaN();
         }
      }
   }

   @XmlElement
   public void setConfiguration(Pose3D configuration)
   {
      SixDoFJointStateBasics.super.setConfiguration(configuration);
   }

   @XmlElement
   public void setAngularVelocity(Vector3D angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   @XmlElement
   public void setLinearVelocity(Vector3D linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   @XmlElement
   public void setAngularAcceleration(Vector3D angularAcceleration)
   {
      this.angularAcceleration.set(angularAcceleration);
   }

   @XmlElement
   public void setLinearAcceleration(Vector3D linearAcceleration)
   {
      this.linearAcceleration.set(linearAcceleration);
   }

   @XmlElement
   public void setTorque(Vector3D torque)
   {
      this.torque.set(torque);
   }

   @XmlElement
   public void setForce(Vector3D force)
   {
      this.force.set(force);
   }

   @Override
   public Pose3D getConfiguration()
   {
      return configuration;
   }

   @Override
   public Vector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public Vector3D getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public Vector3D getAngularAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public Vector3D getLinearAcceleration()
   {
      return linearAcceleration;
   }

   @Override
   public Vector3D getTorque()
   {
      return torque;
   }

   @Override
   public Vector3D getForce()
   {
      return force;
   }

   @Override
   public SixDoFJointState copy()
   {
      return new SixDoFJointState(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, configuration);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocity);
      bits = EuclidHashCodeTools.addToHashCode(bits, linearVelocity);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularAcceleration);
      bits = EuclidHashCodeTools.addToHashCode(bits, linearAcceleration);
      bits = EuclidHashCodeTools.addToHashCode(bits, torque);
      bits = EuclidHashCodeTools.addToHashCode(bits, force);
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

      SixDoFJointState other = (SixDoFJointState) object;

      if (!Objects.equals(configuration, other.configuration))
         return false;
      if (!Objects.equals(angularVelocity, other.angularVelocity))
         return false;
      if (!Objects.equals(linearVelocity, other.linearVelocity))
         return false;
      if (!Objects.equals(angularAcceleration, other.angularAcceleration))
         return false;
      if (!Objects.equals(linearAcceleration, other.linearAcceleration))
         return false;
      if (!Objects.equals(torque, other.torque))
         return false;
      if (!Objects.equals(force, other.force))
         return false;

      return true;
   }
}
