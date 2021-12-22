package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;

public interface SphericalJointStateReadOnly extends JointStateReadOnly
{
   QuaternionReadOnly getConfiguration();

   Vector3DReadOnly getVelocity();

   Vector3DReadOnly getAcceleration();

   Vector3DReadOnly getEffort();

   @Override
   default int getConfigurationSize()
   {
      return 4;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointOrientation().set(getConfiguration());
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      getConfiguration().get(startRow, configurationToPack);
      return startRow + getConfigurationSize();
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointAngularVelocity().set(getVelocity());
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      getVelocity().get(startRow, velocityToPack);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointAngularAcceleration().set(getAcceleration());
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      getAcceleration().get(startRow, accelerationToPack);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointTorque().set(getEffort());
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      getEffort().get(startRow, effortToPack);
      return startRow + getDegreesOfFreedom();
   }
}
