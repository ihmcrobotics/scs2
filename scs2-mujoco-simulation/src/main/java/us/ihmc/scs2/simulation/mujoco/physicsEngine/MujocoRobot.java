package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.HashMap;
import java.util.Map;

import org.bytedeco.javacpp.DoublePointer;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoMultiBodyRobot.JointAddress;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * SCS2-side wrapper that owns the mecano {@link Robot} and the parallel {@link MujocoMultiBodyRobot}.
 * Implements the per-step state I/O: read controller torques into MuJoCo, then read MuJoCo state
 * back into mecano so SCS2 frames, sensors, and YoVariables update correctly.
 */
public class MujocoRobot extends RobotExtension
{
   private final MujocoMultiBodyRobot mujocoMultiBodyRobot;
   private final YoRegistry yoRegistry;

   private final Quaternion quaternion = new Quaternion();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularAcceleration = new Vector3D();
   private final Vector3D position = new Vector3D();

   // Sensor plumbing: provide per-body external wrench (from MuJoCo cfrc_ext) and per-body
   // spatial acceleration (from mecano's analytic calculator, driven by joint qdd pulled out of
   // MuJoCo). RobotPhysicsOutput hands these to SimWrenchSensor.update / SimIMUSensor.update.
   private final SpatialAccelerationCalculator accelerationCalculator;
   private final RigidBodyWrenchRegistry wrenchRegistry = new RigidBodyWrenchRegistry();
   private final RobotPhysicsOutput physicsOutput;
   private final Map<Integer, SimRigidBodyBasics> mecanoBodyByMujocoId = new HashMap<>();

   private final Wrench scratchWrench = new Wrench();
   private final Wrench tmpExternalWrench = new Wrench();

   // F/T moment-arm correction: cfrc_ext is at the body CoM, we register it at the body origin.
   private final FrameVector3D comOffsetWorld = new FrameVector3D();
   private final Vector3D forceWorld = new Vector3D();
   private final Vector3D torqueShift = new Vector3D();

   // Finite-diff scratch for floating-root spatial acceleration.
   private final Quaternion prevFloatingOrientation = new Quaternion();
   private final Vector3D prevFloatingTwistLinear = new Vector3D();
   private final Vector3D prevFloatingTwistAngular = new Vector3D();
   private final Vector3D scratchPrevAngularInCurrentFrame = new Vector3D();
   private final Vector3D scratchPrevLinearInCurrentFrame = new Vector3D();
   private boolean havePreviousFloatingState = false;

   public MujocoRobot(Robot robot, YoRegistry physicsRegistry, MujocoMultiBodyRobot mujocoMultiBodyRobot)
   {
      super(robot, physicsRegistry);
      this.mujocoMultiBodyRobot = mujocoMultiBodyRobot;
      this.yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());
      robot.getRegistry().addChild(yoRegistry);

      // doVelocityTerms=true: include Coriolis and centripetal acceleration so IMU linear-acc
      // readings include the v x w term. Matches BulletRobotPhysics.
      accelerationCalculator = new SpatialAccelerationCalculator(robot.getRootBody(), robot.getInertialFrame(), true);
      accelerationCalculator.setGravitionalAcceleration(-9.81);
      physicsOutput = new RobotPhysicsOutput(accelerationCalculator, null, wrenchRegistry, null);

      // Cache the mecano body for each MuJoCo body id once so per-tick cfrc_ext readout is an
      // O(1) lookup rather than a name resolve through the SCS2 joint tree.
      for (SimJointBasics joint : robot.getAllJoints())
      {
         SimRigidBodyBasics body = joint.getSuccessor();
         if (body == null)
            continue;
         int bodyId = mujocoMultiBodyRobot.getBodyId(body.getName());
         if (bodyId >= 0)
            mecanoBodyByMujocoId.put(bodyId, body);
      }
   }

   public MujocoMultiBodyRobot getMujocoMultiBodyRobot()
   {
      return mujocoMultiBodyRobot;
   }

   /**
    * Pack per-body external wrenches from {@code mjData.cfrc_ext} into the registry, invalidate
    * the spatial-acceleration cache, then walk every considered joint and update its sensor
    * auxiliary data ({@code SimIMUSensor}, {@code SimWrenchSensor}, etc.).
    *
    * <p>{@code cfrc_ext[bodyId * 6 .. +5]} is laid out as {@code [torque (rot)|force (trans)]}
    * (per {@code mjdata.h}'s "rotation:translation format" comment), expressed at the body's CoM
    * in world frame. The registry stores the wrench at the body-fixed-frame origin, so we apply
    * the standard wrench shift {@code T_origin = T_CoM + (CoM - origin) x F} -- a known ~10 N*m
    * bias on Alex feet without this correction (CoM offset ~5 cm, contact force ~250 N).
    */
   public void updateSensors(DoublePointer cfrcExt)
   {
      ReferenceFrame worldFrame = getRobot().getInertialFrame();
      wrenchRegistry.reset();
      for (Map.Entry<Integer, SimRigidBodyBasics> entry : mecanoBodyByMujocoId.entrySet())
      {
         int base = entry.getKey() * 6;
         SimRigidBodyBasics body = entry.getValue();

         double torqueAtCoMX = cfrcExt.get(base);
         double torqueAtCoMY = cfrcExt.get(base + 1);
         double torqueAtCoMZ = cfrcExt.get(base + 2);
         double forceX = cfrcExt.get(base + 3);
         double forceY = cfrcExt.get(base + 4);
         double forceZ = cfrcExt.get(base + 5);

         // CoM offset is in body-fixed frame; rotating into world gives (CoM - origin) in world.
         comOffsetWorld.setIncludingFrame(body.getInertia().getCenterOfMassOffset());
         comOffsetWorld.changeFrame(worldFrame);
         forceWorld.set(forceX, forceY, forceZ);
         torqueShift.cross(comOffsetWorld, forceWorld);

         scratchWrench.setToZero(body.getBodyFixedFrame(), worldFrame);
         scratchWrench.getAngularPart().set(torqueAtCoMX + torqueShift.getX(),
                                            torqueAtCoMY + torqueShift.getY(),
                                            torqueAtCoMZ + torqueShift.getZ());
         scratchWrench.getLinearPart().set(forceX, forceY, forceZ);
         wrenchRegistry.addWrench(body, scratchWrench);
      }

      accelerationCalculator.reset();

      for (SimJointBasics joint : getJointsToConsider())
         joint.getAuxiliaryData().update(physicsOutput);
   }

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
            tmpExternalWrench.setIncludingFrame(wp.getWrench());
            tmpExternalWrench.changeFrame(worldFrame);
            // MuJoCo xfrc layout: (Fx, Fy, Fz, Tx, Ty, Tz) in world frame at body CoM.
            xfrcApplied.put(base + 0, xfrcApplied.get(base + 0) + tmpExternalWrench.getLinearPartX());
            xfrcApplied.put(base + 1, xfrcApplied.get(base + 1) + tmpExternalWrench.getLinearPartY());
            xfrcApplied.put(base + 2, xfrcApplied.get(base + 2) + tmpExternalWrench.getLinearPartZ());
            xfrcApplied.put(base + 3, xfrcApplied.get(base + 3) + tmpExternalWrench.getAngularPartX());
            xfrcApplied.put(base + 4, xfrcApplied.get(base + 4) + tmpExternalWrench.getAngularPartY());
            xfrcApplied.put(base + 5, xfrcApplied.get(base + 5) + tmpExternalWrench.getAngularPartZ());
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
      }
   }

   private int pullCount = 0;

   /**
    * Read MuJoCo {@code qpos} / {@code qvel} / {@code qacc} back into the mecano joint state so
    * SCS2 frames, twists, and accelerations are consistent with the MuJoCo simulation. Joint
    * accelerations are pulled so the {@link SpatialAccelerationCalculator} downstream of
    * {@link #updateSensors} returns meaningful IMU linear-acceleration readings.
    */
   public void pullStateFromMujoco(double currentTime,
                                   double dt,
                                   DoublePointer qpos,
                                   DoublePointer qvel,
                                   DoublePointer qacc)
   {
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
            // Snapshot previous orientation and joint twist so we can finite-diff the spatial
            // acceleration in the joint frame. Direct copy of mjData.qacc is the wrong recipe:
            // MuJoCo's free-joint qacc is a conventional kinematic acceleration in world frame,
            // but mecano expects a spatial acceleration in the body frame (with the v x w
            // Coriolis term baked in). BulletRobotLinkRoot.computeJointAcceleration uses the
            // same finite-diff + cross-product recipe we replicate below.
            boolean canFiniteDiff = havePreviousFloatingState && dt > 0.0;
            if (canFiniteDiff)
            {
               prevFloatingOrientation.set(floating.getJointPose().getOrientation());
               prevFloatingTwistLinear.set(floating.getJointTwist().getLinearPart());
               prevFloatingTwistAngular.set(floating.getJointTwist().getAngularPart());
            }

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

            if (canFiniteDiff)
            {
               // Rotate previous twist components from previous joint frame into current joint
               // frame, then subtract from current to get the body-frame finite-diff. Standard
               // moving-frame derivative recipe.
               prevFloatingOrientation.transform(prevFloatingTwistAngular, scratchPrevAngularInCurrentFrame);
               floating.getJointPose().getOrientation().inverseTransform(scratchPrevAngularInCurrentFrame);
               angularAcceleration.sub(floating.getJointTwist().getAngularPart(), scratchPrevAngularInCurrentFrame);
               angularAcceleration.scale(1.0 / dt);

               prevFloatingOrientation.transform(prevFloatingTwistLinear, scratchPrevLinearInCurrentFrame);
               floating.getJointPose().getOrientation().inverseTransform(scratchPrevLinearInCurrentFrame);
               linearAcceleration.sub(floating.getJointTwist().getLinearPart(), scratchPrevLinearInCurrentFrame);
               linearAcceleration.scale(1.0 / dt);

               floating.getJointAcceleration().getAngularPart().set(angularAcceleration);
               floating.getJointAcceleration().getLinearPart().set(linearAcceleration);
               // Convert conventional acceleration to spatial: a^spatial_linear = a_linear + v x w.
               floating.getJointAcceleration().addCrossToLinearPart(floating.getJointTwist().getLinearPart(),
                                                                     floating.getJointTwist().getAngularPart());
            }
            else
            {
               floating.getJointAcceleration().setToZero();
            }
            havePreviousFloatingState = true;
         }
         else if (joint instanceof OneDoFJointBasics oneDoF)
         {
            oneDoF.setQ(qpos.get(address.qposadr));
            oneDoF.setQd(qvel.get(address.qveladr));
            // For 1-DoF joints qdd is a scalar second derivative -- no frame ambiguity, so we
            // trust MuJoCo's qacc directly. (The freejoint above can't do this.)
            oneDoF.setQdd(qacc.get(address.qveladr));
         }
      }

      updateFrames();
   }
}
