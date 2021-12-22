package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface PlanarJointStateBasics extends PlanarJointStateReadOnly, JointStateBasics
{
   void setConfiguration(double pitch, double positionX, double positionZ);

   void setVelocity(double pitchVelocity, double linearVelocityX, double linearVelocityZ);

   void setAcceleration(double pitchAcceleration, double linearAccelerationX, double linearAccelerationZ);

   void setEffort(double torqueY, double forceX, double forceZ);

   default void set(PlanarJointStateReadOnly other)
   {
      clear();
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
         setConfiguration(other.getPitch(), other.getPositionX(), other.getPositionZ());
      if (other.hasOutputFor(JointStateType.VELOCITY))
         setVelocity(other.getPitchVelocity(), other.getLinearVelocityX(), other.getLinearVelocityZ());
      if (other.hasOutputFor(JointStateType.ACCELERATION))
         setAcceleration(other.getPitchAcceleration(), other.getLinearAccelerationX(), other.getLinearAccelerationZ());
      if (other.hasOutputFor(JointStateType.EFFORT))
         setEffort(other.getTorqueY(), other.getForceX(), other.getForceZ());
   }

   default void setConfiguration(Pose3DReadOnly configuration)
   {
      setConfiguration(configuration.getOrientation(), configuration.getPosition());
   }

   default void setConfiguration(Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      setConfiguration(orientation.getPitch(), position.getX(), position.getZ());
   }

   default void setVelocity(SpatialVectorReadOnly velocity)
   {
      setVelocity(velocity.getAngularPart(), velocity.getLinearPart());
   }

   default void setVelocity(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearVelocity)
   {
      setVelocity(angularVelocity.getY(), linearVelocity.getX(), linearVelocity.getZ());
   }

   default void setAcceleration(SpatialVectorReadOnly acceleration)
   {
      setAcceleration(acceleration.getAngularPart(), acceleration.getLinearPart());
   }

   default void setAcceleration(Vector3DReadOnly angularAcceleration, Vector3DReadOnly linearAcceleration)
   {
      setAcceleration(angularAcceleration.getY(), linearAcceleration.getX(), linearAcceleration.getZ());
   }

   default void setEffort(SpatialVectorReadOnly effort)
   {
      setEffort(effort.getAngularPart(), effort.getLinearPart());
   }

   default void setEffort(Vector3DReadOnly torque, Vector3DReadOnly force)
   {
      setEffort(torque.getY(), force.getX(), force.getZ());
   }

   @Override
   default void setConfiguration(JointReadOnly joint)
   {
      setConfiguration(((PlanarJointReadOnly) joint).getJointPose());
   }

   @Override
   default int setConfiguration(int startRow, DMatrix configuration)
   {
      setConfiguration(configuration.get(startRow++, 0), configuration.get(startRow++, 0), configuration.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setVelocity(JointReadOnly joint)
   {
      setVelocity(((PlanarJointReadOnly) joint).getJointTwist());
   }

   @Override
   default int setVelocity(int startRow, DMatrix velocity)
   {
      setVelocity(velocity.get(startRow++, 0), velocity.get(startRow++, 0), velocity.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setAcceleration(JointReadOnly joint)
   {
      setAcceleration(((PlanarJointReadOnly) joint).getJointAcceleration());
   }

   @Override
   default int setAcceleration(int startRow, DMatrix acceleration)
   {
      setAcceleration(acceleration.get(startRow++, 0), acceleration.get(startRow++, 0), acceleration.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setEffort(JointReadOnly joint)
   {
      setEffort(((PlanarJointReadOnly) joint).getJointWrench());
   }

   @Override
   default int setEffort(int startRow, DMatrix effort)
   {
      setEffort(effort.get(startRow++, 0), effort.get(startRow++, 0), effort.get(startRow++, 0));
      return startRow;
   }
}
