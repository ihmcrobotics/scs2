package us.ihmc.scs2.definition.state.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface SixDoFJointStateBasics extends JointStateBasics, SixDoFJointStateReadOnly
{
   void setConfiguration(Orientation3DReadOnly orientation, Tuple3DReadOnly position);

   void setVelocity(Vector3DReadOnly angularVelocity, Vector3DReadOnly linearVelocity);

   void setAcceleration(Vector3DReadOnly angularAcceleration, Vector3DReadOnly linearAcceleration);

   void setEffort(Vector3DReadOnly torque, Vector3DReadOnly force);

   default void set(SixDoFJointStateReadOnly other)
   {
      clear();
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
         setConfiguration(other.getConfiguration());
      if (other.hasOutputFor(JointStateType.VELOCITY))
         setVelocity(other.getAngularVelocity(), other.getLinearVelocity());
      if (other.hasOutputFor(JointStateType.ACCELERATION))
         setAcceleration(other.getAngularAcceleration(), other.getLinearAcceleration());
      if (other.hasOutputFor(JointStateType.EFFORT))
         setEffort(other.getTorque(), other.getForce());
   }

   default void setConfiguration(Pose3DReadOnly configuration)
   {
      setConfiguration(configuration.getOrientation(), configuration.getPosition());
   }

   default void setVelocity(SpatialVectorReadOnly velocity)
   {
      setVelocity(velocity.getAngularPart(), velocity.getLinearPart());
   }

   default void setAcceleration(SpatialVectorReadOnly acceleration)
   {
      setAcceleration(acceleration.getAngularPart(), acceleration.getLinearPart());
   }

   default void setEffort(SpatialVectorReadOnly effort)
   {
      setEffort(effort.getAngularPart(), effort.getLinearPart());
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
}
