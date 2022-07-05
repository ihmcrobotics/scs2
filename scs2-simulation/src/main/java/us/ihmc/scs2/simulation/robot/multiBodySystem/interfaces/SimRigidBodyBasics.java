package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;

public interface SimRigidBodyBasics extends RigidBodyBasics, SimRigidBodyReadOnly
{
   @Override
   SimJointBasics getParentJoint();

   @Override
   default Iterable<? extends SimRigidBodyBasics> subtreeIterable()
   {
      return new RigidBodyIterable<>(SimRigidBodyBasics.class, null, this);
   }

   @Override
   default Iterable<? extends SimJointBasics> childrenSubtreeIterable()
   {
      return new JointIterable<>(SimJointBasics.class, null, getChildrenJoints());
   }

   @Override
   default Stream<? extends SimRigidBodyBasics> subtreeStream()
   {
      return SubtreeStreams.from(SimRigidBodyBasics.class, this);
   }

   @Override
   default List<? extends SimRigidBodyBasics> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default SimRigidBodyBasics[] subtreeArray()
   {
      return subtreeStream().toArray(SimRigidBodyBasics[]::new);
   }

   default void updateAuxiliaryDataRecursively(RobotPhysicsOutput physicsOutput)
   {
      List<JointBasics> childrenJoints = getChildrenJoints();
      for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
      {
         ((SimJointBasics) childrenJoints.get(childIndex)).updateAuxiliaryDataRecursively(physicsOutput);
      }
   }
}
