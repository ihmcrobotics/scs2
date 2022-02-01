package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface SixDoFJointStateBasics extends JointStateBasics, SixDoFJointStateReadOnly
{
   @Override
   Pose3DBasics getConfiguration();

   @Override
   Vector3DBasics getAngularVelocity();

   @Override
   Vector3DBasics getLinearVelocity();

   @Override
   Vector3DBasics getAngularAcceleration();

   @Override
   Vector3DBasics getLinearAcceleration();

   @Override
   Vector3DBasics getTorque();

   @Override
   Vector3DBasics getForce();

   default void set(SixDoFJointStateReadOnly other)
   {
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
      {
         setConfiguration(other.getConfiguration());
      }
      else
      {
         getConfiguration().setToNaN();
      }

      if (other.hasOutputFor(JointStateType.VELOCITY))
      {
         setVelocity(other.getAngularVelocity(), other.getLinearVelocity());
      }
      else
      {
         getAngularVelocity().setToNaN();
         getLinearVelocity().setToNaN();
      }

      if (other.hasOutputFor(JointStateType.ACCELERATION))
      {
         setAcceleration(other.getAngularAcceleration(), other.getLinearAcceleration());
      }
      else
      {
         getAngularAcceleration().setToNaN();
         getLinearAcceleration().setToNaN();
      }

      if (other.hasOutputFor(JointStateType.EFFORT))
      {
         setEffort(other.getTorque(), other.getForce());
      }
      else
      {
         getTorque().setToNaN();
         getForce().setToNaN();
      }
   }

   default void setConfiguration(Pose3DReadOnly configuration)
   {
      setConfiguration(configuration.getOrientation(), configuration.getPosition());
   }

   default void setConfiguration(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      if (orientation != null)
         getConfiguration().getOrientation().set(orientation);
      else
         getConfiguration().getOrientation().setToZero();

      if (position != null)
         getConfiguration().getPosition().set(position);
      else
         getConfiguration().getPosition().setToZero();
   }

   default void setVelocity(SpatialVectorReadOnly velocity)
   {
      setVelocity(velocity.getAngularPart(), velocity.getLinearPart());
   }

   default void setVelocity(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearVelocity)
   {
      if (angularVelocity != null)
         getAngularVelocity().set(angularVelocity);
      else
         getAngularVelocity().setToZero();

      if (linearVelocity != null)
         getLinearVelocity().set(linearVelocity);
      else
         getLinearVelocity().setToZero();
   }

   default void setAcceleration(SpatialVectorReadOnly acceleration)
   {
      setAcceleration(acceleration.getAngularPart(), acceleration.getLinearPart());
   }

   default void setAcceleration(Vector3DReadOnly angularAcceleration, Vector3DReadOnly linearAcceleration)
   {
      if (angularAcceleration != null)
         getAngularAcceleration().set(angularAcceleration);
      else
         getAngularAcceleration().setToZero();

      if (linearAcceleration != null)
         getLinearAcceleration().set(linearAcceleration);
      else
         getLinearAcceleration().setToZero();
   }

   default void setEffort(SpatialVectorReadOnly effort)
   {
      setEffort(effort.getAngularPart(), effort.getLinearPart());
   }

   default void setEffort(Vector3DReadOnly torque, Vector3DReadOnly force)
   {
      if (torque != null)
         getTorque().set(torque);
      else
         getTorque().setToZero();

      if (force != null)
         getForce().set(force);
      else
         getForce().setToZero();
   }

   @Override
   default void setConfiguration(JointReadOnly joint)
   {
      setConfiguration(((SixDoFJointReadOnly) joint).getJointPose());
   }

   @Override
   default void setVelocity(JointReadOnly joint)
   {
      setVelocity(((SixDoFJointReadOnly) joint).getJointTwist());
   }

   @Override
   default void setAcceleration(JointReadOnly joint)
   {
      setAcceleration(((SixDoFJointReadOnly) joint).getJointAcceleration());
   }

   @Override
   default void setEffort(JointReadOnly joint)
   {
      setEffort(((SixDoFJointReadOnly) joint).getJointWrench());
   }

   @Override
   default int setConfiguration(int startRow, DMatrix configuration)
   {
      getConfiguration().getOrientation().set(startRow, configuration);
      getConfiguration().getPosition().set(startRow + 4, configuration);
      return startRow + getConfigurationSize();
   }

   @Override
   default int setVelocity(int startRow, DMatrix velocity)
   {
      getAngularVelocity().set(startRow, velocity);
      getLinearVelocity().set(startRow + 3, velocity);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default int setAcceleration(int startRow, DMatrix acceleration)
   {
      getAngularAcceleration().set(startRow, acceleration);
      getLinearAcceleration().set(startRow + 3, acceleration);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default int setEffort(int startRow, DMatrix effort)
   {
      getTorque().set(startRow, effort);
      getForce().set(startRow + 3, effort);
      return startRow + getDegreesOfFreedom();
   }
}
