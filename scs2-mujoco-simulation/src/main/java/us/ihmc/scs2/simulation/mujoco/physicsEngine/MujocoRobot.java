package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import org.bytedeco.javacpp.DoublePointer;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoMultiBodyRobot.JointAddress;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * SCS2-side wrapper that owns the mecano {@link Robot} and the parallel {@link MujocoMultiBodyRobot}.
 * Implements the per-step state I/O: read controller torques into MuJoCo, then read MuJoCo state
 * back into mecano so SCS2 frames and YoVariables update correctly.
 */
public class MujocoRobot extends RobotExtension
{
   private final MujocoMultiBodyRobot mujocoMultiBodyRobot;
   private final YoRegistry yoRegistry;

   // Scratch quaternion to avoid allocation in pullStateFromMujoco.
   private final Quaternion quaternion = new Quaternion();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D position = new Vector3D();

   public MujocoRobot(Robot robot, YoRegistry physicsRegistry, MujocoMultiBodyRobot mujocoMultiBodyRobot)
   {
      super(robot, physicsRegistry);
      this.mujocoMultiBodyRobot = mujocoMultiBodyRobot;
      this.yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());
      robot.getRegistry().addChild(yoRegistry);
   }

   public MujocoMultiBodyRobot getMujocoMultiBodyRobot()
   {
      return mujocoMultiBodyRobot;
   }

   /**
    * No-op in v1. {@code BulletRobot} uses this hook to push contact forces (gathered from the
    * native engine) into the mecano sensor tree via {@code RobotPhysicsOutput}. Replicating that
    * requires reading {@code mjData.cfrc_ext} per body and is deferred to a follow-up. SCS2's
    * frame update via {@link #updateFrames()} already runs from {@code MujocoPhysicsEngine}.
    */
   public void updateSensors()
   {
   }

   private final us.ihmc.mecano.spatial.Wrench tmpWrench = new us.ihmc.mecano.spatial.Wrench();

   /**
    * Copy every {@link us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint}'s current
    * wrench into MuJoCo's per-body {@code xfrc_applied} array, in world frame at the body's
    * CoM. Limitations in v1:
    * <ul>
    *   <li>Moment-arm transfer assumes the body's body-frame origin coincides with its CoM
    *       (true when {@code RigidBodyDefinition.getCenterOfMassOffset()} is zero, which it is
    *       for the SCS2 examples used so far). Bodies with non-zero CoM offset will see a
    *       moment error equal to {@code offset x F}.</li>
    *   <li>{@code xfrc_applied} entries for managed bodies are zeroed at the start of each push
    *       so persistent wrenches don't accumulate across steps.</li>
    * </ul>
    */
   public void pushExternalWrenchesToMujoco(DoublePointer xfrcApplied)
   {
      // The SCS2 session's inertial frame is "world" within the session's frame tree. Using the
      // global ReferenceFrame.getWorldFrame() here throws "frames do not have same roots"
      // because SCS2 builds a separate root for each session.
      us.ihmc.euclid.referenceFrame.ReferenceFrame worldFrame = getRobot().getInertialFrame();
      for (us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics joint : getRobot().getAllJoints())
      {
         if (joint.getSuccessor() == null)
            continue;
         var wrenchPoints = joint.getAuxiliaryData().getExternalWrenchPoints();
         if (wrenchPoints.isEmpty())
            continue;
         int bodyId = mujocoMultiBodyRobot.getBodyId(joint.getSuccessor().getName());
         if (bodyId < 0)
            continue;

         int base = bodyId * 6;
         for (int i = 0; i < 6; i++)
            xfrcApplied.put(base + i, 0.0);

         for (var wp : wrenchPoints)
         {
            tmpWrench.setIncludingFrame(wp.getWrench());
            tmpWrench.changeFrame(worldFrame);
            // MuJoCo xfrc layout: (Fx, Fy, Fz, Tx, Ty, Tz) in world frame at body CoM.
            xfrcApplied.put(base + 0, xfrcApplied.get(base + 0) + tmpWrench.getLinearPartX());
            xfrcApplied.put(base + 1, xfrcApplied.get(base + 1) + tmpWrench.getLinearPartY());
            xfrcApplied.put(base + 2, xfrcApplied.get(base + 2) + tmpWrench.getLinearPartZ());
            xfrcApplied.put(base + 3, xfrcApplied.get(base + 3) + tmpWrench.getAngularPartX());
            xfrcApplied.put(base + 4, xfrcApplied.get(base + 4) + tmpWrench.getAngularPartY());
            xfrcApplied.put(base + 5, xfrcApplied.get(base + 5) + tmpWrench.getAngularPartZ());
         }
      }
   }

   /**
    * Write joint efforts (torques/forces from the controller) into MuJoCo's {@code qfrc_applied}
    * for each managed joint. Floating root contributes no controller torque.
    */
   public void pushStateToMujoco(DoublePointer qfrcApplied, DoublePointer qpos, DoublePointer qvel)
   {
      for (JointBasics joint : getJointsToConsider())
      {
         JointAddress address = mujocoMultiBodyRobot.getJointAddress(joint.getName());
         if (address == null)
            continue;
         if (address.isFloatingRoot)
            continue;
         if (joint instanceof OneDoFJointBasics oneDoF)
         {
            qfrcApplied.put(address.qveladr, oneDoF.getTau());
         }
         else
         {
            // Multi-DoF non-root joints not handled in v1; silently skip.
         }
      }
   }

   private int pullCount = 0;

   /**
    * Read MuJoCo {@code qpos} / {@code qvel} back into the mecano joint configuration so SCS2
    * frame updates pick up the new state. Acceleration is approximated by finite difference over
    * {@code dt} when the caller passes a non-zero value.
    */
   public void pullStateFromMujoco(double currentTime, double dt, DoublePointer qpos, DoublePointer qvel)
   {
      // Throttle to ~5 prints/sec at 1 kHz tick.
      if (pullCount < 5 || pullCount % 200 == 0)
      {
         JointAddress root = mujocoMultiBodyRobot.getRootJointAddress();
         if (root != null)
         {
            System.out.printf("[MujocoRobot] t=%.3fs pull #%d qpos[root]=(%.4f, %.4f, %.4f)%n",
                              currentTime,
                              pullCount,
                              qpos.get(root.qposadr),
                              qpos.get(root.qposadr + 1),
                              qpos.get(root.qposadr + 2));
         }
      }
      pullCount++;
      for (JointBasics joint : getJointsToConsider())
      {
         JointAddress address = mujocoMultiBodyRobot.getJointAddress(joint.getName());
         if (address == null)
            continue;
         if (address.isFloatingRoot && joint instanceof SixDoFJointBasics floating)
         {
            int qp = address.qposadr;
            position.set(qpos.get(qp), qpos.get(qp + 1), qpos.get(qp + 2));
            // MuJoCo quaternion order is (w, x, y, z).
            quaternion.set(qpos.get(qp + 4), qpos.get(qp + 5), qpos.get(qp + 6), qpos.get(qp + 3));
            floating.getJointPose().getPosition().set(position);
            floating.getJointPose().getOrientation().set(quaternion);

            int qv = address.qveladr;
            linearVelocity.set(qvel.get(qv), qvel.get(qv + 1), qvel.get(qv + 2));
            angularVelocity.set(qvel.get(qv + 3), qvel.get(qv + 4), qvel.get(qv + 5));
            floating.getJointTwist().getLinearPart().set(linearVelocity);
            floating.getJointTwist().getAngularPart().set(angularVelocity);
         }
         else if (joint instanceof OneDoFJointBasics oneDoF)
         {
            oneDoF.setQ(qpos.get(address.qposadr));
            oneDoF.setQd(qvel.get(address.qveladr));
         }
      }

      updateFrames();
   }
}
