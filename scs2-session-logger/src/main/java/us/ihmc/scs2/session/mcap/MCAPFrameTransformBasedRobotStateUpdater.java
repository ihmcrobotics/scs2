package us.ihmc.scs2.session.mcap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.session.mcap.MCAPFrameTransformManager.YoFoxGloveFrameTransform;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to update the robot state based on the frame transforms.
 */
public class MCAPFrameTransformBasedRobotStateUpdater implements RobotStateUpdater
{
   private final List<Runnable> jointStateUpdaters = new ArrayList<>();
   private final boolean coversAllJoints;

   public MCAPFrameTransformBasedRobotStateUpdater(Robot robot, MCAPFrameTransformManager frameTransformManager, MCAPJointStateManager jointStateManager,
                                                    MCAPOdometryManager odometryManager, YoLong currentTimestamp)
   {
      // Counts how many joints *should* end up with a working updater below, so it can be compared against how many
      // actually did (see coversAllJoints below). Only OneDoFJointBasics/SixDoFJointBasics are ever driven by /tf in
      // the first place, so other joint types (e.g. fixed joints) are skipped and don't count either way.
      int attemptedJointCount = 0;

      for (JointBasics joint : robot.getAllJoints())
      {
         if (!(joint instanceof OneDoFJointBasics) && !(joint instanceof SixDoFJointBasics))
            continue;
         attemptedJointCount++;

         // Pre-existing behavior: a joint whose /tf transform can't be found is silently skipped rather than
         // failing construction outright, so jointStateUpdaters can end up smaller than attemptedJointCount -
         // that gap is exactly what coversAllJoints() surfaces to the caller below.
         String successorName = joint.getSuccessor().getName();
         YoFoxGloveFrameTransform transform = frameTransformManager.getTransformFromSanitizedName(successorName);
         if (transform == null)
         {
            LogTools.error("No transform found for " + successorName);
            continue;
         }

         if (joint instanceof OneDoFJointBasics oneDoFJoint)
         {
            // OneDoFJointStateUpdater needs the parent joint's transform to root to compute the joint's local configuration,
            // unlike SixDoFJointStateUpdater below, which only needs the successor's transform to its immediate TF parent.
            String predecessorName = joint.getPredecessor().getName();
            YoFoxGloveFrameTransform parentJointTransform = frameTransformManager.getTransformFromSanitizedName(predecessorName);
            if (parentJointTransform == null)
            {
               LogTools.error("No transform found for " + predecessorName);
               continue;
            }
            YoDouble qd = jointStateManager == null ? null : jointStateManager.getVelocity(oneDoFJoint.getName());
            jointStateUpdaters.add(new OneDoFJointStateUpdater(oneDoFJoint, transform, parentJointTransform, qd));
         }
         else if (joint instanceof SixDoFJointBasics sixDoFJoint)
         {
            jointStateUpdaters.add(new SixDoFJointStateUpdater(sixDoFJoint, transform, odometryManager, currentTimestamp));
         }
      }

      // true only if every eligible joint above actually resolved a transform - a partial match (some joints
      // updated, others silently skipped) is treated the same as a total miss, since a partially frozen robot is
      // just as broken as a fully frozen one.
      coversAllJoints = jointStateUpdaters.size() == attemptedJointCount;
   }

   /**
    * @return {@code false} if any joint failed to resolve a {@code /tf} transform (e.g. the file's {@code /tf}
    *       channel exists but was never actually published to) - {@link MCAPLogFileReader#createRobotStateUpdater}
    *       uses this to fall back to another strategy instead of returning an updater that would leave the robot
    *       frozen.
    *       <p>
    *       This is the fix for a real case: a log whose {@code /tf} channel existed but was mostly empty, which
    *       used to make this class silently construct itself with almost no working joint updaters (a frozen
    *       robot) with no way for the caller to know it had come up empty. {@code createRobotStateUpdater} now
    *       reads this flag and falls through to {@link MCAPControllerStateBasedRobotStateUpdater} (or the mujoco-
    *       based updater) instead - "try {@code /tf} first, but if it doesn't actually cover the robot, don't
    *       settle for a frozen updater." This check is generic: it only counts joints against whatever {@link Robot}
    *       was actually loaded, nothing here is tied to a specific robot definition.
    *       </p>
    */
   public boolean coversAllJoints()
   {
      return coversAllJoints;
   }

   @Override
   public void updateRobotState()
   {
      for (Runnable jointStateUpdater : jointStateUpdaters)
      {
         jointStateUpdater.run();
      }
   }

   /**
    * Prefers the floating-base joint's exact twist from a standard {@code nav_msgs/Odometry} channel
    * ({@link MCAPOdometryManager}) when the file has one. There is no ROS message that standardizes floating-base
    * joint velocity the way {@code /tf} standardizes pose (unlike OneDoF joints, which get exact velocity from the
    * standard {@code sensor_msgs/JointState.velocity[]} field via {@link MCAPJointStateManager}), so files without
    * odometry fall back to finite-differencing consecutive {@code /tf} poses instead. That fallback is a naive
    * sequential cache: jumping the log-scrubber to an arbitrary position will show one stale velocity reading
    * immediately after the jump, self-correcting on the next tick.
    */
   public static class SixDoFJointStateUpdater implements Runnable
   {
      private final SixDoFJointBasics joint;
      private final YoFoxGloveFrameTransform transform;
      private final MCAPOdometryManager odometryManager;
      private final YoLong currentTimestamp;

      private final Vector3D previousTranslation = new Vector3D();
      private final Quaternion previousOrientation = new Quaternion();
      private long previousTimestamp = Long.MIN_VALUE;

      private final Quaternion currentOrientation = new Quaternion();
      private final Vector3D linearVelocityWorld = new Vector3D();
      private final Vector3D linearVelocityBody = new Vector3D();
      private final Vector3D angularVelocityBody = new Vector3D();

      public SixDoFJointStateUpdater(SixDoFJointBasics joint, YoFoxGloveFrameTransform transform, MCAPOdometryManager odometryManager,
                                      YoLong currentTimestamp)
      {
         this.joint = joint;
         this.transform = transform;
         this.odometryManager = odometryManager;
         this.currentTimestamp = currentTimestamp;
      }

      @Override
      public void run()
      {
         RigidBodyTransformReadOnly transformToParentJoint = transform.getTransformToParent();
         joint.setJointConfiguration(transformToParentJoint);

         if (odometryManager != null && odometryManager.hasTwist())
         {
            joint.getJointTwist()
                 .getAngularPart()
                 .set(odometryManager.getAngularVelocityX(), odometryManager.getAngularVelocityY(), odometryManager.getAngularVelocityZ());
            joint.getJointTwist()
                 .getLinearPart()
                 .set(odometryManager.getLinearVelocityX(), odometryManager.getLinearVelocityY(), odometryManager.getLinearVelocityZ());
            return;
         }

         Tuple3DReadOnly currentTranslation = transformToParentJoint.getTranslation();
         currentOrientation.set(transformToParentJoint.getRotation());

         long timestamp = currentTimestamp.getValue();
         long dtNanos = timestamp - previousTimestamp;

         if (previousTimestamp != Long.MIN_VALUE && dtNanos > 0)
         {
            // Quaternions have double cover (q and -q represent the same rotation), and nothing guarantees the
            // rotation-matrix-to-quaternion conversion above picks a temporally-consistent sign tick to tick.
            // QuaternionTools.finiteDifference doesn't correct for that itself: if the sign flips between
            // previousOrientation and currentOrientation, it computes the "long way around" rotation instead of
            // the (correct) near-zero one, producing a spurious huge angular velocity spike. Force the shortest
            // path by flipping to the same hemisphere as previousOrientation before differencing.
            if (previousOrientation.dot(currentOrientation) < 0.0)
               currentOrientation.negate();

            double dt = dtNanos / 1.0e9;

            // World-frame linear delta, rotated into the body frame joint.getJointTwist() expects - mirrors
            // MCAPMujocoBasedRobotStateUpdater.SixDoFJointStateUpdater's handling of a frameBeforeJoint-expressed velocity.
            linearVelocityWorld.sub(currentTranslation, previousTranslation);
            linearVelocityWorld.scale(1.0 / dt);
            currentOrientation.inverseTransform(linearVelocityWorld, linearVelocityBody);
            joint.getJointTwist().getLinearPart().set(linearVelocityBody);

            // QuaternionTools.finiteDifference already expresses the result in the current orientation's local
            // (body-fixed) coordinates, matching joint.getJointTwist()'s convention directly.
            QuaternionTools.finiteDifference(previousOrientation, currentOrientation, dt, angularVelocityBody);
            joint.getJointTwist().getAngularPart().set(angularVelocityBody);
         }

         previousTranslation.set(currentTranslation);
         previousOrientation.set(currentOrientation);
         previousTimestamp = timestamp;
      }
   }

   public static class OneDoFJointStateUpdater implements Runnable
   {
      private final OneDoFJointBasics joint;
      private final YoFoxGloveFrameTransform transform;
      private final YoFoxGloveFrameTransform parentJointTransform;
      private final YoDouble qd;
      private final RigidBodyTransform jointConfiguration = new RigidBodyTransform();

      public OneDoFJointStateUpdater(OneDoFJointBasics joint, YoFoxGloveFrameTransform transform, YoFoxGloveFrameTransform parentJointTransform,
                                      YoDouble qd)
      {
         this.joint = joint;
         this.transform = transform;
         this.parentJointTransform = parentJointTransform;
         this.qd = qd;
      }

      @Override
      public void run()
      {
         RigidBodyTransformReadOnly beforeJointTransform = joint.getFrameBeforeJoint().getTransformToParent();
         RigidBodyTransformReadOnly transformParentJointToRoot = parentJointTransform.getTransformToRoot();
         RigidBodyTransformReadOnly transformToRoot = transform.getTransformToRoot();

         jointConfiguration.setIdentity();
         jointConfiguration.setAndInvert(beforeJointTransform);
         jointConfiguration.multiplyInvertOther(transformParentJointToRoot);
         jointConfiguration.multiply(transformToRoot);
         joint.setJointConfiguration(jointConfiguration);

         if (qd != null)
            joint.setQd(qd.getValue());
      }
   }
}
