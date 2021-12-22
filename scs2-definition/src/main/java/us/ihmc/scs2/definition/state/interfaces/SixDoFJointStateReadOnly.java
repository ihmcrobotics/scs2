package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;

public interface SixDoFJointStateReadOnly extends JointStateReadOnly
{
   Pose3DReadOnly getConfiguration();

   Vector3DReadOnly getAngularVelocity();

   Vector3DReadOnly getLinearVelocity();

   Vector3DReadOnly getAngularAcceleration();

   Vector3DReadOnly getLinearAcceleration();

   Vector3DReadOnly getTorque();

   Vector3DReadOnly getForce();

   @Override
   default int getConfigurationSize()
   {
      return 7;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 6;
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      getConfiguration().getOrientation().get(startRow, configurationToPack);
      startRow += 4;
      getConfiguration().getPosition().get(startRow, configurationToPack);
      return startRow + 3;
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      getAngularVelocity().get(startRow, velocityToPack);
      startRow += 3;
      getLinearVelocity().get(startRow, velocityToPack);
      return startRow + 3;
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      getAngularAcceleration().get(startRow, accelerationToPack);
      startRow += 3;
      getLinearAcceleration().get(startRow, accelerationToPack);
      return startRow + 3;
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      getTorque().get(startRow, effortToPack);
      startRow += 3;
      getForce().get(startRow, effortToPack);
      return startRow + 3;
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).setJointConfiguration(getConfiguration());
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointTwist().set(getAngularVelocity(), getLinearVelocity());
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointAcceleration().set(getAngularAcceleration(), getLinearAcceleration());
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointWrench().set(getTorque(), getForce());
   }
}
