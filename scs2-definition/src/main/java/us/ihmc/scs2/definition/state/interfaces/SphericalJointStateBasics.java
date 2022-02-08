package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;

public interface SphericalJointStateBasics extends SphericalJointStateReadOnly, JointStateBasics
{
   @Override
   QuaternionBasics getOrientation();

   @Override
   Vector3DBasics getAngularVelocity();

   @Override
   Vector3DBasics getAngularAcceleration();

   @Override
   Vector3DBasics getTorque();

   default void clear()
   {
      getOrientation().setToNaN();
      getAngularVelocity().setToNaN();
      getAngularAcceleration().setToNaN();
      getTorque().setToNaN();
   }

   default void setConfiguration(Orientation3DReadOnly configuration)
   {
      getOrientation().set(configuration);
   }

   default void setVelocity(Vector3DReadOnly angularVelocity)
   {
      getAngularVelocity().set(angularVelocity);
   }

   default void setAcceleration(Vector3DReadOnly angularAcceleration)
   {
      getAngularAcceleration().set(angularAcceleration);
   }

   default void setEffort(Vector3DReadOnly torque)
   {
      getTorque().set(torque);
   }

   default void set(SphericalJointStateReadOnly other)
   {
      setConfiguration(other.getOrientation());
      setVelocity(other.getAngularVelocity());
      setAcceleration(other.getAngularAcceleration());
      setEffort(other.getTorque());
   }

   @Override
   default void setConfiguration(JointReadOnly joint)
   {
      getOrientation().set(((SphericalJointReadOnly) joint).getJointOrientation());
   }

   @Override
   default void setVelocity(JointReadOnly joint)
   {
      getAngularVelocity().set(((SphericalJointReadOnly) joint).getJointAngularVelocity());
   }

   @Override
   default void setAcceleration(JointReadOnly joint)
   {
      getAngularAcceleration().set(((SphericalJointReadOnly) joint).getJointAngularAcceleration());
   }

   @Override
   default void setEffort(JointReadOnly joint)
   {
      getTorque().set(((SphericalJointReadOnly) joint).getJointTorque());
   }

   @Override
   default int setConfiguration(int startRow, DMatrix configuration)
   {
      getOrientation().set(startRow, configuration);
      return startRow + getConfigurationSize();
   }

   @Override
   default int setVelocity(int startRow, DMatrix velocity)
   {
      getAngularVelocity().set(startRow, velocity);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default int setAcceleration(int startRow, DMatrix acceleration)
   {
      getAngularAcceleration().set(startRow, acceleration);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default int setEffort(int startRow, DMatrix effort)
   {
      getTorque().set(startRow, effort);
      return startRow + getDegreesOfFreedom();
   }
}
