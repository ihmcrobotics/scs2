package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives a robot's state from a decoded {@code persona_proto.ControllerState} protobuf channel (see
 * {@link YoMCAPProtobufMessage}), for files where the standard {@code /tf}/{@code /joint_states} channels are
 * empty and this is the only channel carrying complete robot state.
 * <p>
 * Reads the root pose/twist from {@code robot_state.floating_base_state} and each one-DoF joint's
 * position/velocity/torque from {@code robot_state.joint_states.<jointName>} - the latter only exists once
 * {@link YoMCAPProtobufMessage} has resolved that protobuf {@code map} field against a sample message.
 * </p>
 */
public class MCAPControllerStateBasedRobotStateUpdater implements RobotStateUpdater
{
   private static final String CONTROLLER_STATE_SCHEMA_NAME = "persona_proto.ControllerState";

   public static boolean isRobotControllerStateMessage(Robot robot, YoMCAPProtobufMessage message)
   {
      if (!CONTROLLER_STATE_SCHEMA_NAME.equals(message.getDescriptor().getFullName()))
         return false;

      YoRegistry jointStates = jointStatesRegistry(message);
      if (jointStates == null || floatingBaseStateRegistry(message) == null)
         return false;

      for (JointReadOnly joint : robot.getAllJoints())
      {
         if (joint instanceof OneDoFJointBasics && jointStates.getChild(joint.getName()) == null)
            return false;
      }

      return true;
   }

   private final List<Runnable> jointStateUpdaters = new ArrayList<>();

   public MCAPControllerStateBasedRobotStateUpdater(Robot robot, YoMCAPProtobufMessage message)
   {
      YoRegistry floatingBaseState = floatingBaseStateRegistry(message);
      YoRegistry jointStates = jointStatesRegistry(message);

      SimFloatingJointBasics rootJoint = robot.getFloatingRootJoint();

      if (rootJoint != null)
      {
         if (rootJoint instanceof SixDoFJointBasics sixDoFJoint)
            jointStateUpdaters.add(new SixDoFJointStateUpdater(sixDoFJoint, floatingBaseState));
         else
            throw new UnsupportedOperationException("Cannot handle root joint type: " + rootJoint.getClass().getSimpleName());
      }

      for (JointReadOnly joint : robot.getAllJoints())
      {
         if (joint == rootJoint)
            continue;

         if (joint instanceof OneDoFJointBasics oneDoFJoint)
         {
            YoRegistry jointRegistry = jointStates.getChild(joint.getName());
            if (jointRegistry != null)
               jointStateUpdaters.add(new OneDoFJointStateUpdater(oneDoFJoint, jointRegistry));
         }
      }
   }

   @Override
   public void updateRobotState()
   {
      for (Runnable jointStateUpdater : jointStateUpdaters)
      {
         jointStateUpdater.run();
      }
   }

   private static YoRegistry floatingBaseStateRegistry(YoMCAPProtobufMessage message)
   {
      YoRegistry robotState = message.getRegistry().getChild("robot_state");
      return robotState == null ? null : robotState.getChild("floating_base_state");
   }

   private static YoRegistry jointStatesRegistry(YoMCAPProtobufMessage message)
   {
      YoRegistry robotState = message.getRegistry().getChild("robot_state");
      return robotState == null ? null : robotState.getChild("joint_states");
   }

   private static class SixDoFJointStateUpdater implements Runnable
   {
      private final SixDoFJointBasics joint;
      private final YoPoint3D position;
      private final YoQuaternion orientation;
      private final YoPose3D pose;
      private final YoVector3D angularVelocityBody;
      private final YoVector3D linearVelocityWorld;

      private final Vector3D linearVelocityBodyScratch = new Vector3D();

      SixDoFJointStateUpdater(SixDoFJointBasics joint, YoRegistry floatingBaseState)
      {
         this.joint = joint;

         YoRegistry poseWorld = floatingBaseState.getChild("pose_world");
         YoRegistry positionRegistry = poseWorld.getChild("position");
         YoRegistry orientationRegistry = poseWorld.getChild("orientation");
         position = new YoPoint3D((YoDouble) positionRegistry.findVariable("x"),
                                   (YoDouble) positionRegistry.findVariable("y"),
                                   (YoDouble) positionRegistry.findVariable("z"));
         orientation = new YoQuaternion((YoDouble) orientationRegistry.findVariable("x"),
                                         (YoDouble) orientationRegistry.findVariable("y"),
                                         (YoDouble) orientationRegistry.findVariable("z"),
                                         (YoDouble) orientationRegistry.findVariable("w"));
         pose = new YoPose3D(position, orientation);

         YoRegistry angularVelocityRegistry = floatingBaseState.getChild("angular_velocity_body");
         angularVelocityBody = angularVelocityRegistry == null ? null : new YoVector3D((YoDouble) angularVelocityRegistry.findVariable("x"),
                                                                                        (YoDouble) angularVelocityRegistry.findVariable("y"),
                                                                                        (YoDouble) angularVelocityRegistry.findVariable("z"));

         YoRegistry linearVelocityRegistry = floatingBaseState.getChild("linear_velocity_world");
         linearVelocityWorld = linearVelocityRegistry == null ? null : new YoVector3D((YoDouble) linearVelocityRegistry.findVariable("x"),
                                                                                       (YoDouble) linearVelocityRegistry.findVariable("y"),
                                                                                       (YoDouble) linearVelocityRegistry.findVariable("z"));
      }

      @Override
      public void run()
      {
         // pose_world/angular_velocity_body/linear_velocity_world are all proto3 optional sub-messages: unlike a
         // scalar field (which just reads back its zero default when unset), an *absent* sub-message resets every
         // YoVariable inside it to NaN (see YoMCAPProtobufMessage's scalar deserializer). linear_velocity_world in
         // particular isn't published on every tick. Feeding a NaN into the joint's twist trips SimSixDoFJoint's
         // own validity check, so skip a component set entirely rather than propagate NaN - same "leave the last
         // known good value in place" trade-off MCAPFrameTransformBasedRobotStateUpdater's finite-difference
         // fallback already makes when data momentarily isn't available.
         if (isFinite(position) && isFinite(orientation))
            joint.getJointPose().set(pose);

         Orientation3DReadOnly currentOrientation = pose.getOrientation();

         if (angularVelocityBody != null && isFinite(angularVelocityBody))
            joint.getJointTwist().getAngularPart().set(angularVelocityBody);

         if (linearVelocityWorld != null && isFinite(linearVelocityWorld))
         {
            currentOrientation.inverseTransform(linearVelocityWorld, linearVelocityBodyScratch);
            joint.getJointTwist().getLinearPart().set(linearVelocityBodyScratch);
         }
      }

      private static boolean isFinite(Tuple3DReadOnly tuple)
      {
         return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
      }

      private static boolean isFinite(QuaternionReadOnly quaternion)
      {
         return Double.isFinite(quaternion.getX()) && Double.isFinite(quaternion.getY()) && Double.isFinite(quaternion.getZ())
                && Double.isFinite(quaternion.getS());
      }
   }

   private static class OneDoFJointStateUpdater implements Runnable
   {
      private final OneDoFJointBasics joint;
      private final YoDouble q;
      private final YoDouble qd;
      private final YoDouble tau;

      OneDoFJointStateUpdater(OneDoFJointBasics joint, YoRegistry jointRegistry)
      {
         this.joint = joint;
         q = (YoDouble) jointRegistry.findVariable("position");
         qd = (YoDouble) jointRegistry.findVariable("velocity");
         tau = (YoDouble) jointRegistry.findVariable("torque");
      }

      @Override
      public void run()
      {
         if (q != null)
            joint.setQ(q.getValue());
         if (qd != null)
            joint.setQd(qd.getValue());
         if (tau != null)
            joint.setTau(tau.getValue());
      }
   }
}
