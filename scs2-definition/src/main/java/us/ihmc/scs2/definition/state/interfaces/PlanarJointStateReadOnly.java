package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;

public interface PlanarJointStateReadOnly extends JointStateReadOnly
{
   public double getPitch();

   public double getPositionX();

   public double getPositionZ();

   public double getPitchVelocity();

   public double getLinearVelocityX();

   public double getLinearVelocityZ();

   public double getPitchAcceleration();

   public double getLinearAccelerationX();

   public double getLinearAccelerationZ();

   public double getTorqueY();

   public double getForceX();

   public double getForceZ();

   @Override
   default int getConfigurationSize()
   {
      return 3;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      configurationToPack.set(startRow++, 0, getPitch());
      configurationToPack.set(startRow++, 0, getPositionX());
      configurationToPack.set(startRow++, 0, getPositionZ());
      return startRow;
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      velocityToPack.set(startRow++, 0, getPitchVelocity());
      velocityToPack.set(startRow++, 0, getLinearVelocityX());
      velocityToPack.set(startRow++, 0, getLinearVelocityZ());
      return startRow;
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      accelerationToPack.set(startRow++, 0, getPitchAcceleration());
      accelerationToPack.set(startRow++, 0, getLinearAccelerationX());
      accelerationToPack.set(startRow++, 0, getLinearAccelerationZ());
      return startRow;
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      effortToPack.set(startRow++, 0, getTorqueY());
      effortToPack.set(startRow++, 0, getForceX());
      effortToPack.set(startRow++, 0, getForceZ());
      return startRow;
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      Pose3DBasics jointPose = ((PlanarJointBasics) jointToUpdate).getJointPose();
      jointPose.getOrientation().setToPitchOrientation(getPitch());
      jointPose.getPosition().setX(getPositionX());
      jointPose.getPosition().setZ(getPositionZ());
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      FixedFrameTwistBasics jointTwist = ((PlanarJointBasics) jointToUpdate).getJointTwist();
      jointTwist.setAngularPartY(getPitchVelocity());
      jointTwist.setLinearPartX(getLinearVelocityX());
      jointTwist.setLinearPartZ(getLinearVelocityZ());
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      FixedFrameSpatialAccelerationBasics jointAcceleration = ((PlanarJointBasics) jointToUpdate).getJointAcceleration();
      jointAcceleration.setAngularPartY(getPitchAcceleration());
      jointAcceleration.setLinearPartX(getLinearAccelerationX());
      jointAcceleration.setLinearPartZ(getLinearAccelerationZ());
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      FixedFrameWrenchBasics jointWrench = ((PlanarJointBasics) jointToUpdate).getJointWrench();
      jointWrench.setAngularPartY(getTorqueY());
      jointWrench.setLinearPartX(getForceX());
      jointWrench.setLinearPartZ(getForceZ());
   }
}
