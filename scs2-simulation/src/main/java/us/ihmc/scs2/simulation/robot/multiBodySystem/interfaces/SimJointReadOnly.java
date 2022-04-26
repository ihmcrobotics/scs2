package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface SimJointReadOnly extends JointReadOnly
{
   boolean isPinned();

   YoRegistry getRegistry();

   @Override
   SimRigidBodyReadOnly getPredecessor();

   @Override
   SimRigidBodyReadOnly getSuccessor();

   TwistReadOnly getJointDeltaTwist();

   default void getSuccessorDeltaTwist(TwistBasics successorDeltaTwistToPack)
   {
      successorDeltaTwistToPack.setIncludingFrame(getJointDeltaTwist());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      successorDeltaTwistToPack.setBaseFrame(predecessorFrame);
      successorDeltaTwistToPack.setBodyFrame(successorFrame);
      successorDeltaTwistToPack.changeFrame(successorFrame);
   }

   default void getPredecessorDeltaTwist(TwistBasics predecessorDeltaTwistToPack)
   {
      predecessorDeltaTwistToPack.setIncludingFrame(getJointDeltaTwist());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      predecessorDeltaTwistToPack.setBaseFrame(predecessorFrame);
      predecessorDeltaTwistToPack.setBodyFrame(successorFrame);
      predecessorDeltaTwistToPack.invert();
      predecessorDeltaTwistToPack.changeFrame(predecessorFrame);
   }

   int getJointDeltaVelocity(int rowStart, DMatrix matrixToPack);

   SimJointAuxiliaryData getAuxialiryData();

   @Override
   default Iterable<? extends SimJointReadOnly> subtreeIterable()
   {
      return new JointIterable<>(SimJointBasics.class, null, this);
   }

   @Override
   default Stream<? extends SimJointReadOnly> subtreeStream()
   {
      return SubtreeStreams.from(SimJointReadOnly.class, this);
   }

   @Override
   default List<? extends SimJointReadOnly> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default SimJointReadOnly[] subtreeArray()
   {
      return subtreeStream().toArray(SimJointReadOnly[]::new);
   }
}
