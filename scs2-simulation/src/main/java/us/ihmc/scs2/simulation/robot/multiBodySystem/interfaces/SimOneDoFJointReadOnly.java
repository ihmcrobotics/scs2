package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;

public interface SimOneDoFJointReadOnly extends SimJointReadOnly, OneDoFJointReadOnly
{
   double getDeltaQd();

   @Override
   default void getSuccessorDeltaTwist(TwistBasics deltaTwistToPack)
   {
      deltaTwistToPack.setIncludingFrame(getUnitSuccessorTwist());
      deltaTwistToPack.scale(getDeltaQd());
   }

   @Override
   default void getPredecessorDeltaTwist(TwistBasics deltaTwistToPack)
   {
      deltaTwistToPack.setIncludingFrame(getUnitPredecessorTwist());
      deltaTwistToPack.scale(getQd());
   }

   @Override
   default int getJointDeltaVelocity(int rowStart, DMatrix matrixToPack)
   {
      matrixToPack.set(rowStart, 0, getDeltaQd());
      return rowStart + getDegreesOfFreedom();
   }
}
