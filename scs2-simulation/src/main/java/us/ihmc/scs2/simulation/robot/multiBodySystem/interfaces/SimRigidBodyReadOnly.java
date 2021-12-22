package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.scs2.simulation.collision.CollidableHolder;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRigidBody;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface SimRigidBodyReadOnly extends RigidBodyReadOnly, CollidableHolder
{
   YoRegistry getRegistry();

   @Override
   SimJointReadOnly getParentJoint();

   @Override
   default Iterable<? extends SimRigidBodyReadOnly> subtreeIterable()
   {
      return new RigidBodyIterable<>(SimRigidBodyReadOnly.class, null, this);
   }

   @Override
   default Iterable<? extends SimJointReadOnly> childrenSubtreeIterable()
   {
      return new JointIterable<>(SimJointReadOnly.class, null, getChildrenJoints());
   }

   @Override
   default Stream<? extends SimRigidBodyReadOnly> subtreeStream()
   {
      return SubtreeStreams.from(SimRigidBodyReadOnly.class, this);
   }

   @Override
   default List<? extends SimRigidBodyReadOnly> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default SimRigidBodyReadOnly[] subtreeArray()
   {
      return subtreeStream().toArray(SimRigidBody[]::new);
   }
}
