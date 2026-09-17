package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.HashMap;
import java.util.Map;

import org.bytedeco.javacpp.DoublePointer;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
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

   // Scratch for cacc CoM-to-joint-origin spatial acceleration shift.
   private final Vector3D caccCoMShift = new Vector3D();

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
    * Creates a per-body contact-total group for every body with collision geometry, keyed by MuJoCo
    * body id, in this robot's registry. Populated each tick by the engine's {@link YoMujocoContactPool}.
    */
   public Map<Integer, MujocoBodyContactAggregate> createContactAggregates(ReferenceFrame worldFrame)
   {
      Map<Integer, MujocoBodyContactAggregate> aggregates = new HashMap<>();
      for (Map.Entry<Integer, SimRigidBodyBasics> entry : mecanoBodyByMujocoId.entrySet())
      {
         String bodyName = entry.getValue().getName();
         RigidBodyDefinition bodyDefinition = getRobotDefinition().getRigidBodyDefinition(bodyName);
         if (bodyDefinition == null || bodyDefinition.getCollisionShapeDefinitions().isEmpty())
            continue;
         aggregates.put(entry.getKey(), new MujocoBodyContactAggregate(bodyName, worldFrame, yoRegistry));
      }
      return aggregates;
   }

   /**
    * Pack per-body external wrenches from {@code mjData.cfrc_ext} into the registry, invalidate the
    * spatial-acceleration cache, then update every considered joint's sensor auxiliary data (IMU,
    * wrench sensors, etc.). The cfrc_ext wrench is at the body CoM; it is shifted to the
    * body-fixed-frame origin (a ~10 N*m bias on Alex feet without the shift). See inline notes for
    * the {@code [torque|force]} layout.
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
    * wrench into MuJoCo's per-body {@code xfrc_applied} array, in world frame at the body's CoM.
    *
    * <p>{@code xfrc_applied} layout is {@code [force | torque]} (translation:rotation). This is the
    * exception among MuJoCo's per-body 6-vectors: the spatial arrays {@code cfrc_ext}, {@code cacc},
    * and {@code cvel} use {@code [torque | force]} (rotation:translation), but {@code xfrc_applied}
    * is consumed by {@code mj_applyFT(force, torque, point, ...)} in {@code mj_xfrcAccumulate}, so its
    * first three entries are the force and its last three are the torque.
    *
    * <p>The moment is taken about the body CoM (what {@code xfrc_applied} expects). The wrench is
    * first changed to world frame -- which expresses the moment about the world origin -- then shifted
    * to the CoM: {@code M_CoM = M_worldOrigin - r_CoM x F}.
    *
    * <p>{@code xfrc_applied} entries for managed bodies are zeroed at the start of each push so
    * persistent wrenches don't accumulate across steps.
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
         SimRigidBodyBasics body = joint.getSuccessor();
         int bodyId = mujocoMultiBodyRobot.getBodyId(body.getName());
         if (bodyId < 0)
            continue;

         int base = bodyId * 6;
         for (int i = 0; i < 6; i++)
            xfrcApplied.put(base + i, 0.0);

         // Body CoM position in world — same for all EWPs on this body, so compute once.
         pushBodyComPosWorld.setIncludingFrame(body.getInertia().getCenterOfMassOffset());
         pushBodyComPosWorld.changeFrame(worldFrame);

         for (var wp : wrenchPoints)
         {
            tmpExternalWrench.setIncludingFrame(wp.getWrench());
            // changeFrame on a spatial force moves the moment reference point to the new frame's
            // origin: after this the linear part is F in world, and the angular part is the moment
            // about the WORLD ORIGIN (= M_wp + r_wp x F).
            tmpExternalWrench.changeFrame(worldFrame);

            double fx = tmpExternalWrench.getLinearPartX();
            double fy = tmpExternalWrench.getLinearPartY();
            double fz = tmpExternalWrench.getLinearPartZ();

            // MuJoCo xfrc_applied expects [force | torque] with the torque taken about the body CoM,
            // in world frame. Shift the moment from the world origin to the CoM:
            //   M_CoM = M_worldOrigin + (r_worldOrigin - r_CoM) x F = M_worldOrigin - r_CoM x F.
            // (The earlier version added (r_wp - r_CoM) x F to the world-origin moment, which left a
            // spurious r_wp x F term -- a ~10x phantom roll moment under a chest push.)
            double comX = pushBodyComPosWorld.getX();
            double comY = pushBodyComPosWorld.getY();
            double comZ = pushBodyComPosWorld.getZ();
            double shiftToComX = -(comY * fz - comZ * fy);
            double shiftToComY = -(comZ * fx - comX * fz);
            double shiftToComZ = -(comX * fy - comY * fx);

            // xfrc_applied is [force | torque]: force in slots 0..2, torque in slots 3..5.
            xfrcApplied.put(base + 0, xfrcApplied.get(base + 0) + fx);
            xfrcApplied.put(base + 1, xfrcApplied.get(base + 1) + fy);
            xfrcApplied.put(base + 2, xfrcApplied.get(base + 2) + fz);
            xfrcApplied.put(base + 3, xfrcApplied.get(base + 3) + tmpExternalWrench.getAngularPartX() + shiftToComX);
            xfrcApplied.put(base + 4, xfrcApplied.get(base + 4) + tmpExternalWrench.getAngularPartY() + shiftToComY);
            xfrcApplied.put(base + 5, xfrcApplied.get(base + 5) + tmpExternalWrench.getAngularPartZ() + shiftToComZ);
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

   // Scratch for pushExternalWrenchesToMujoco moment-arm correction.
   private final FramePoint3D pushBodyComPosWorld = new FramePoint3D();

   /**
    * Read MuJoCo {@code qpos} / {@code qvel} / {@code qacc} / {@code cacc} back into the mecano joint
    * state so SCS2 frames, twists, and accelerations stay consistent with the MuJoCo simulation. The
    * floating root uses {@code cacc} (com-based spatial acceleration) rather than finite-differencing
    * the twist; see the inline notes for the (subtle) MuJoCo frame conventions.
    */
   public void pullStateFromMujoco(Vector3DReadOnly gravity,
                                   DoublePointer qpos,
                                   DoublePointer qvel,
                                   DoublePointer qacc,
                                   DoublePointer cacc)
   {
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
            // MuJoCo freejoint qvel convention is split:
            //   qvel[0:3] (linear) -- inertial/WORLD frame
            //   qvel[3:6] (angular) -- BODY frame
            // mecano's SixDoFJoint twist expects both parts in the body (after-joint) frame.
            // So only the linear part needs rotation; angular passes through.
            //   v_body = R^T * v_world
            // Empirically verified: rotating the angular part as well regresses walking time
            // (138 s -> ~20 s in non-perfect-sensor mode), confirming the per-component split.
            quaternion.inverseTransform(linearVelocity);
            floating.getJointTwist().getLinearPart().set(linearVelocity);
            floating.getJointTwist().getAngularPart().set(angularVelocity);

            // Read floating-root acceleration from cacc (com-based spatial acceleration).
            // This is exact and lag-free compared to finite-differencing the joint twist.
            int bodyId = mujocoMultiBodyRobot.getBodyId(floating.getSuccessor().getName());
            if (bodyId >= 0)
            {
               int base = bodyId * 6;
               // cacc layout: [αx αy αz ax ay az] at CoM in world frame.
               // MuJoCo's cacc uses proper-acceleration convention (root reference = -g), so
               // a body at rest reads +9.81 m/s² upward. SpatialAccelerationCalculator already
               // applies the -g reference via setGravitionalAcceleration, which would double-count
               // gravity. Add gravity here to convert back to the Featherstone mathematical
               // convention (= 0 at rest) that the calculator expects.
               angularAcceleration.set(cacc.get(base), cacc.get(base + 1), cacc.get(base + 2));
               linearAcceleration.set(cacc.get(base + 3), cacc.get(base + 4), cacc.get(base + 5));
               linearAcceleration.add(gravity); // world frame: +(-9.81 z) → 9.81 - 9.81 = 0 at rest

               // Rotate both parts from world to body frame.
               quaternion.inverseTransform(angularAcceleration);
               quaternion.inverseTransform(linearAcceleration);

               // Shift spatial linear acceleration from CoM to joint origin (body frame).
               // For spatial accelerations: a_origin = a_CoM + α × r_{CoM→origin}
               //                                      = a_CoM - α × comOffset
               // (no centripetal ω×(ω×r) term — that only appears in classical acceleration)
               caccCoMShift.cross(angularAcceleration, floating.getSuccessor().getInertia().getCenterOfMassOffset());
               linearAcceleration.sub(caccCoMShift);

               floating.getJointAcceleration().getAngularPart().set(angularAcceleration);
               floating.getJointAcceleration().getLinearPart().set(linearAcceleration);
               // cacc is already Featherstone spatial — addCrossToLinearPart is NOT needed.
            }
            else
            {
               floating.getJointAcceleration().setToZero();
            }
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
