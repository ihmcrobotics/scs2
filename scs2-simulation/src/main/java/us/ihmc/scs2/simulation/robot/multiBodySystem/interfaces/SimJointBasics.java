package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;

public interface SimJointBasics extends JointBasics, SimJointReadOnly
{
   void setPinned(boolean isPinned);

   @Override
   SimRigidBodyBasics getPredecessor();

   @Override
   SimRigidBodyBasics getSuccessor();

   void setJointDeltaTwistToZero();

   void setJointDeltaTwist(JointReadOnly other);

   int setJointDeltaVelocity(int rowStart, DMatrix jointDeltaVelocity);

   default void setJointDeltaTwist(TwistReadOnly jointDeltaTwist)
   {
      jointDeltaTwist.checkBodyFrameMatch(getFrameAfterJoint());
      jointDeltaTwist.checkBaseFrameMatch(getFrameBeforeJoint());
      jointDeltaTwist.checkExpressedInFrameMatch(getFrameAfterJoint());
      setJointAngularDeltaVelocity((Vector3DReadOnly) jointDeltaTwist.getAngularPart());
      setJointLinearDeltaVelocity((Vector3DReadOnly) jointDeltaTwist.getLinearPart());
   }

   default void setJointAngularDeltaVelocity(FrameVector3DReadOnly jointAngularDeltaVelocity)
   {
      jointAngularDeltaVelocity.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointAngularDeltaVelocity((Vector3DReadOnly) jointAngularDeltaVelocity);
   }

   default void setJointLinearDeltaVelocity(FrameVector3DReadOnly jointLinearDeltaVelocity)
   {
      jointLinearDeltaVelocity.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointLinearDeltaVelocity((Vector3DReadOnly) jointLinearDeltaVelocity);
   }

   void setJointAngularDeltaVelocity(Vector3DReadOnly jointAngularDeltaVelocity);

   void setJointLinearDeltaVelocity(Vector3DReadOnly jointLinearDeltaVelocity);

   default void addKinematicPoint(KinematicPointDefinition definition)
   {
      getAuxialiryData().addKinematicPoint(definition);
   }

   default void addExternalWrenchPoint(ExternalWrenchPointDefinition definition)
   {
      getAuxialiryData().addExternalWrenchPoint(definition);
   }

   default void addGroundContactPoint(GroundContactPointDefinition definition)
   {
      getAuxialiryData().addGroundContactPoint(definition);
   }

   default void addIMUSensor(IMUSensorDefinition definition)
   {
      getAuxialiryData().addIMUSensor(definition);
   }

   default void addWrenchSensor(WrenchSensorDefinition definition)
   {
      getAuxialiryData().addWrenchSensor(definition);
   }

   @Override
   default Iterable<? extends SimJointBasics> subtreeIterable()
   {
      return new JointIterable<>(SimJointBasics.class, null, this);
   }

   @Override
   default Stream<? extends SimJointBasics> subtreeStream()
   {
      return SubtreeStreams.from(SimJointBasics.class, this);
   }

   @Override
   default List<? extends SimJointBasics> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default SimJointBasics[] subtreeArray()
   {
      return subtreeStream().toArray(SimJointBasics[]::new);
   }
   
   default void updateAuxiliaryDataRecursively(RobotPhysicsOutput physicsOutput)
   {
      getAuxialiryData().update(physicsOutput);

      if (getSuccessor() != null)
      {
         getSuccessor().updateAuxiliaryDataRecursively(physicsOutput);
      }
   }
   
}
