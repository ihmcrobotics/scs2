package us.ihmc.scs2.definition.state.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface SphericalJointStateBasics extends SphericalJointStateReadOnly, JointStateBasics
{
   void setConfiguration(Orientation3DReadOnly configuration);

   void setVelocity(Vector3DReadOnly angularVelocity);

   void setAcceleration(Vector3DReadOnly angularAcceleration);

   void setEffort(Vector3DReadOnly torque);

   default void set(SphericalJointStateReadOnly other)
   {
      clear();
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
         setConfiguration(other.getConfiguration());
      if (other.hasOutputFor(JointStateType.VELOCITY))
         setVelocity(other.getVelocity());
      if (other.hasOutputFor(JointStateType.ACCELERATION))
         setAcceleration(other.getAcceleration());
      if (other.hasOutputFor(JointStateType.EFFORT))
         setEffort(other.getEffort());
   }

   @Override
   default void setConfiguration(JointReadOnly joint)
   {
      SphericalJointReadOnly sphericalJoint = (SphericalJointReadOnly) joint;
      setConfiguration(sphericalJoint.getJointOrientation());
   }

   @Override
   default void setVelocity(JointReadOnly joint)
   {
      SphericalJointReadOnly sphericalJoint = (SphericalJointReadOnly) joint;
      setVelocity(sphericalJoint.getJointAngularVelocity());
   }

   @Override
   default void setAcceleration(JointReadOnly joint)
   {
      SphericalJointReadOnly sphericalJoint = (SphericalJointReadOnly) joint;
      setAcceleration(sphericalJoint.getJointAngularAcceleration());
   }

   @Override
   default void setEffort(JointReadOnly joint)
   {
      SphericalJointReadOnly sphericalJoint = (SphericalJointReadOnly) joint;
      setEffort(sphericalJoint.getJointTorque());
   }
}
