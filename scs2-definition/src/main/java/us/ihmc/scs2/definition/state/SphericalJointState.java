package us.ihmc.scs2.definition.state;

import javax.xml.bind.annotation.XmlElement;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.QuaternionDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateReadOnly;

public class SphericalJointState extends JointStateBase implements SphericalJointStateBasics
{
   private final QuaternionDefinition orientation = new QuaternionDefinition();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D angularAcceleration = new Vector3D();
   private final Vector3D torque = new Vector3D();

   private final DMatrixRMaj temp = new DMatrixRMaj(1, 1);

   public SphericalJointState()
   {
   }

   public SphericalJointState(JointStateReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof SphericalJointStateReadOnly)
      {
         SphericalJointStateBasics.super.set((SphericalJointStateReadOnly) jointStateReadOnly);
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
            orientation.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.VELOCITY))
         {
            jointStateReadOnly.getVelocity(0, temp);
            setVelocity(0, temp);
         }
         else
         {
            angularVelocity.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.ACCELERATION))
         {
            jointStateReadOnly.getAcceleration(0, temp);
            setAcceleration(0, temp);
         }
         else
         {
            angularAcceleration.setToNaN();
         }

         if (jointStateReadOnly.hasOutputFor(JointStateType.EFFORT))
         {
            jointStateReadOnly.getEffort(0, temp);
            setEffort(0, temp);
         }
         else
         {
            torque.setToNaN();
         }
      }
   }

   @XmlElement
   public void setOrientation(QuaternionDefinition orientation)
   {
      this.orientation.set(orientation);
   }

   @XmlElement
   public void setAngularVelocity(Vector3D angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   @XmlElement
   public void setAngularAcceleration(Vector3D angularAcceleration)
   {
      this.angularAcceleration.set(angularAcceleration);
   }

   @XmlElement
   public void setTorque(Vector3D torque)
   {
      this.torque.set(torque);
   }

   @Override
   public QuaternionDefinition getOrientation()
   {
      return orientation;
   }

   @Override
   public Vector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public Vector3D getAngularAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public Vector3D getTorque()
   {
      return torque;
   }

   @Override
   public SphericalJointState copy()
   {
      return new SphericalJointState(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, orientation);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularVelocity);
      bits = EuclidHashCodeTools.addToHashCode(bits, angularAcceleration);
      bits = EuclidHashCodeTools.addToHashCode(bits, torque);
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

      SphericalJointState other = (SphericalJointState) object;

      if (orientation.containsNaN() ? !other.orientation.containsNaN() : !orientation.equals(other.orientation))
         return false;
      if (angularVelocity.containsNaN() ? !other.angularVelocity.containsNaN() : !angularVelocity.equals(other.angularVelocity))
         return false;
      if (angularAcceleration.containsNaN() ? !other.angularAcceleration.containsNaN() : !angularAcceleration.equals(other.angularAcceleration))
         return false;
      if (torque.containsNaN() ? !other.torque.containsNaN() : !torque.equals(other.torque))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      String ret = "Spherical joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", orientaiton: " + orientation.toStringAsYawPitchRoll();
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", angular velocity: " + angularVelocity;
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", angular acceleration: " + angularAcceleration;
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", torque: " + torque;
      return ret;
   }
}
