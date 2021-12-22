package us.ihmc.scs2.definition.state;

import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateReadOnly;

public class SixDoFJointState extends JointStateBase implements SixDoFJointStateBasics
{
   private final Set<JointStateType> availableStates = EnumSet.noneOf(JointStateType.class);
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
   }

   public SixDoFJointState(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      setConfiguration(orientation, position);
   }

   public SixDoFJointState(JointStateReadOnly other)
   {
      set(other);
   }

   @Override
   public void clear()
   {
      availableStates.clear();
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
      availableStates.addAll(other.availableStates);
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
   public void setConfiguration(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      availableStates.add(JointStateType.CONFIGURATION);

      if (orientation != null)
         configuration.getOrientation().set(orientation);
      else
         configuration.getOrientation().setToZero();

      if (position != null)
         configuration.getPosition().set(position);
      else
         configuration.getPosition().setToZero();
   }

   @Override
   public int setConfiguration(int startRow, DMatrix configuration)
   {
      this.configuration.getOrientation().set(startRow, configuration);
      this.configuration.getPosition().set(startRow + 4, configuration);
      availableStates.add(JointStateType.CONFIGURATION);
      return startRow + getConfigurationSize();
   }

   @Override
   public void setVelocity(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearVelocity)
   {
      availableStates.add(JointStateType.VELOCITY);
      if (angularVelocity == null)
         this.angularVelocity.setToZero();
      else
         this.angularVelocity.set(angularVelocity);
      if (linearVelocity == null)
         this.linearVelocity.setToZero();
      else
         this.linearVelocity.set(linearVelocity);
   }

   @Override
   public int setVelocity(int startRow, DMatrix velocity)
   {
      angularVelocity.set(startRow, velocity);
      linearVelocity.set(startRow + 3, velocity);
      availableStates.add(JointStateType.VELOCITY);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setAcceleration(Vector3DReadOnly angularAcceleration, Vector3DReadOnly linearAcceleration)
   {
      availableStates.add(JointStateType.ACCELERATION);
      if (angularAcceleration != null)
         this.angularAcceleration.set(angularAcceleration);
      else
         this.angularAcceleration.setToZero();
      if (linearAcceleration != null)
         this.linearAcceleration.set(linearAcceleration);
      else
         this.linearAcceleration.setToZero();
   }

   @Override
   public int setAcceleration(int startRow, DMatrix acceleration)
   {
      angularAcceleration.set(startRow, acceleration);
      linearAcceleration.set(startRow + 3, acceleration);
      availableStates.add(JointStateType.ACCELERATION);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setEffort(Vector3DReadOnly torque, Vector3DReadOnly force)
   {
      availableStates.add(JointStateType.EFFORT);
      if (torque != null)
         this.torque.set(torque);
      else
         this.torque.setToZero();
      if (force != null)
         this.force.set(force);
      else
         this.force.setToZero();
   }

   @Override
   public int setEffort(int startRow, DMatrix effort)
   {
      torque.set(startRow, effort);
      force.set(startRow + 3, effort);
      availableStates.add(JointStateType.EFFORT);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      return availableStates.contains(query);
   }

   @Override
   public Pose3DReadOnly getConfiguration()
   {
      return configuration;
   }

   @Override
   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public Vector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public Vector3DReadOnly getAngularAcceleration()
   {
      return angularAcceleration;
   }

   @Override
   public Vector3DReadOnly getLinearAcceleration()
   {
      return linearAcceleration;
   }

   @Override
   public Vector3DReadOnly getTorque()
   {
      return torque;
   }

   @Override
   public Vector3DReadOnly getForce()
   {
      return force;
   }

   @Override
   public SixDoFJointState copy()
   {
      return new SixDoFJointState(this);
   }
}
