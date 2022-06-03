package us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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
import us.ihmc.scs2.simulation.robot.sensors.SimIMUSensor;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.scs2.simulation.robot.trackers.KinematicPoint;

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

   default KinematicPoint addKinematicPoint(String name)
   {
      return getAuxialiryData().addKinematicPoint(name);
   }

   default KinematicPoint addKinematicPoint(String name, Tuple3DReadOnly offset)
   {
      return getAuxialiryData().addKinematicPoint(name, offset);
   }

   default KinematicPoint addKinematicPoint(KinematicPointDefinition definition)
   {
      return getAuxialiryData().addKinematicPoint(definition);
   }

   default ExternalWrenchPoint addExternalWrenchPoint(String name)
   {
      return getAuxialiryData().addExternalWrenchPoint(name);
   }

   default ExternalWrenchPoint addExternalWrenchPoint(String name, Tuple3DReadOnly offset)
   {
      return getAuxialiryData().addExternalWrenchPoint(name, offset);
   }

   default ExternalWrenchPoint addExternalWrenchPoint(ExternalWrenchPointDefinition definition)
   {
      return getAuxialiryData().addExternalWrenchPoint(definition);
   }

   default GroundContactPoint addGroundContactPoint(String name)
   {
      return getAuxialiryData().addGroundContactPoint(name);
   }

   default GroundContactPoint addGroundContactPoint(String name, Tuple3DReadOnly offset)
   {
      return getAuxialiryData().addGroundContactPoint(name, offset);
   }

   default GroundContactPoint addGroundContactPoint(GroundContactPointDefinition definition)
   {
      return getAuxialiryData().addGroundContactPoint(definition);
   }

   default SimIMUSensor addIMUSensor(String name)
   {
      return getAuxialiryData().addIMUSensor(name);
   }

   default SimIMUSensor addIMUSensor(String name, Tuple3DReadOnly offset)
   {
      return getAuxialiryData().addIMUSensor(name, offset);
   }

   default SimIMUSensor addIMUSensor(IMUSensorDefinition definition)
   {
      return getAuxialiryData().addIMUSensor(definition);
   }

   default SimWrenchSensor addWrenchSensor(String name)
   {
      return getAuxialiryData().addWrenchSensor(name);
   }

   default SimWrenchSensor addWrenchSensor(String name, Tuple3DReadOnly offset)
   {
      return getAuxialiryData().addWrenchSensor(name, offset);
   }

   default SimWrenchSensor addWrenchSensor(WrenchSensorDefinition definition)
   {
      return getAuxialiryData().addWrenchSensor(definition);
   }

   @Override
   default void updateFrame()
   {
      JointBasics.super.updateFrame();
      getAuxialiryData().updateFrames();
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
}
