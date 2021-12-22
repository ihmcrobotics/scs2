package us.ihmc.scs2.definition.state;

import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SphericalJointStateReadOnly;

public class SphericalJointState extends JointStateBase implements SphericalJointStateBasics
{
   private final Set<JointStateType> availableStates = EnumSet.noneOf(JointStateType.class);
   private final Quaternion configuration = new Quaternion();
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
   public void clear()
   {
      availableStates.clear();
   }

   public void set(SphericalJointState other)
   {
      configuration.set(other.configuration);
      angularVelocity.set(other.angularVelocity);
      angularAcceleration.set(other.angularAcceleration);
      torque.set(other.torque);
      availableStates.addAll(other.availableStates);
   }

   @Override
   public void set(JointStateReadOnly jointStateReadOnly)
   {
      if (jointStateReadOnly instanceof SphericalJointState)
      {
         set((SphericalJointState) jointStateReadOnly);
      }
      else if (jointStateReadOnly instanceof SphericalJointStateReadOnly)
      {
         SphericalJointStateBasics.super.set((SphericalJointStateReadOnly) jointStateReadOnly);
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
   public void setConfiguration(Orientation3DReadOnly configuration)
   {
      this.configuration.set(configuration);
      availableStates.add(JointStateType.CONFIGURATION);
   }

   @Override
   public int setConfiguration(int startRow, DMatrix configuration)
   {
      this.configuration.set(startRow, configuration);
      availableStates.add(JointStateType.CONFIGURATION);
      return startRow + getConfigurationSize();
   }

   @Override
   public void setVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
      availableStates.add(JointStateType.VELOCITY);
   }

   @Override
   public int setVelocity(int startRow, DMatrix velocity)
   {
      this.angularVelocity.set(startRow, velocity);
      availableStates.add(JointStateType.VELOCITY);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setAcceleration(Vector3DReadOnly angularAcceleration)
   {
      this.angularAcceleration.set(angularAcceleration);
      availableStates.add(JointStateType.ACCELERATION);
   }

   @Override
   public int setAcceleration(int startRow, DMatrix acceleration)
   {
      this.angularAcceleration.set(startRow, acceleration);
      availableStates.add(JointStateType.ACCELERATION);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setEffort(Vector3DReadOnly torque)
   {
      this.torque.set(torque);
      availableStates.add(JointStateType.EFFORT);
   }

   @Override
   public int setEffort(int startRow, DMatrix effort)
   {
      this.torque.set(startRow, effort);
      availableStates.add(JointStateType.EFFORT);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      return availableStates.contains(query);
   }

   @Override
   public Quaternion getConfiguration()
   {
      return configuration;
   }

   @Override
   public Vector3D getVelocity()
   {
      return angularVelocity;
   }

   @Override
   public Vector3D getAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public Vector3D getEffort()
   {
      return torque;
   }

   @Override
   public SphericalJointState copy()
   {
      return new SphericalJointState(this);
   }
}
