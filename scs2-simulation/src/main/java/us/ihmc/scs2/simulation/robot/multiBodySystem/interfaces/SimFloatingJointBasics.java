package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;

public interface SimFloatingJointBasics extends SimFloatingJointReadOnly, SimJointBasics, FloatingJointBasics
{
   @Override
   FixedFrameTwistBasics getJointDeltaTwist();

   @Override
   default void setJointDeltaTwistToZero()
   {
      getJointDeltaTwist().setToZero();
   }

   @Override
   default void setJointAngularDeltaVelocity(Vector3DReadOnly jointAngularDeltaVelocity)
   {
      getJointDeltaTwist().getAngularPart().set(jointAngularDeltaVelocity);
   }

   @Override
   default void setJointLinearDeltaVelocity(Vector3DReadOnly jointLinearDeltaVelocity)
   {
      getJointDeltaTwist().getLinearPart().set(jointLinearDeltaVelocity);
   }
}
