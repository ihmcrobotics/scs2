package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

public interface SixDoFJointStateBasics extends JointStateBasics, SixDoFJointStateReadOnly
{
   @Override
   QuaternionBasics getOrientation();

   @Override
   Point3DBasics getPosition();

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

   @Override
   default void clear()
   {
      getOrientation().setToNaN();
      getPosition().setToNaN();
      getAngularVelocity().setToNaN();
      getLinearVelocity().setToNaN();
      getAngularAcceleration().setToNaN();
      getLinearAcceleration().setToNaN();
      getTorque().setToNaN();
      getForce().setToNaN();
   }

   default void set(SixDoFJointStateReadOnly other)
   {
      getOrientation().set(other.getOrientation());
      getPosition().set(other.getPosition());
      getAngularVelocity().set(other.getAngularVelocity());
      getLinearVelocity().set(other.getLinearVelocity());
      getAngularAcceleration().set(other.getAngularAcceleration());
      getLinearAcceleration().set(other.getLinearAcceleration());
      getTorque().set(other.getTorque());
      getForce().set(other.getForce());
   }

   default void setConfiguration(Pose3DReadOnly configuration)
   {
      getOrientation().set(configuration.getOrientation());
      getPosition().set(configuration.getPosition());
   }

   default void setConfiguration(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      if (orientation != null)
         getOrientation().set(orientation);
      else
         getOrientation().setToZero();

      if (position != null)
         getPosition().set(position);
      else
         getPosition().setToZero();
   }

   default void setVelocity(SpatialVectorReadOnly velocity)
   {
      getAngularVelocity().set(velocity.getAngularPart());
      getLinearVelocity().set(velocity.getLinearPart());
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
      getAngularAcceleration().set(acceleration.getAngularPart());
      getLinearAcceleration().set(acceleration.getLinearPart());
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
      getTorque().set(effort.getAngularPart());
      getForce().set(effort.getLinearPart());
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
      getOrientation().set(startRow, configuration);
      getPosition().set(startRow + 4, configuration);
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
