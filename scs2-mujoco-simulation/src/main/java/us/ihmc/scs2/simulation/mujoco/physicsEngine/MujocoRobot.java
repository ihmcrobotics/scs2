package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.bytedeco.javacpp.BoolPointer;
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
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateReadOnly;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
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
   /** Populated only under JOINT_SERVO; keyed by SCS2 joint name, iterated in registration order. */
   private final Map<String, MujocoJointActuation> jointActuationByName = new LinkedHashMap<>();
   private final List<MujocoJointActuation> jointActuationList = new ArrayList<>();
   /** Parallel to {@link #jointActuationList}: the mecano joint and its DoF address, cached to keep the per-tick loop free of lookups. */
   private final List<OneDoFJointBasics> actuatedJoints = new ArrayList<>();
   /** Parallel to {@link #jointActuationList}: where the controller publishes each joint's low-level command. */
   private final List<OneDoFJointStateReadOnly> actuatedJointOutputs = new ArrayList<>();
   private int[] actuatedJointDofAddresses = new int[0];

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
      // Seeded with standard gravity only so the calculator is usable before the first step;
      // pullStateFromMujoco overwrites it with the session's gravity on every tick.
      accelerationCalculator.setGravitionalAcceleration(0.0, 0.0, -9.81);
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

      {
         for (SimJointBasics joint : getJointsToConsider())
         {
            if (!(joint instanceof OneDoFJointBasics))
               continue;
            int actuatorIndex = mujocoMultiBodyRobot.getActuatorIndex(joint.getName());
            if (actuatorIndex < 0)
               continue;
            MujocoJointActuation actuation = new MujocoJointActuation(joint.getName(), actuatorIndex, yoRegistry);
            jointActuationByName.put(joint.getName(), actuation);
            jointActuationList.add(actuation);
            actuatedJoints.add((OneDoFJointBasics) joint);
            actuatedJointOutputs.add(getControllerManager().getControllerOutput().getOneDoFJointOutput(joint.getName()));
         }

         actuatedJointDofAddresses = new int[jointActuationList.size()];
         for (int i = 0; i < jointActuationList.size(); i++)
         {
            JointAddress address = mujocoMultiBodyRobot.getJointAddress(jointActuationList.get(i).getJointName());
            actuatedJointDofAddresses[i] = address == null ? -1 : address.qveladr;
         }
      }
   }

   /**
    * The low-level command block for a joint, or {@code null} when the joint has no actuator (the
    * free root, welded subtrees, and any joint type the MJCF builder cannot map).
    */
   public MujocoJointActuation getJointActuation(String jointName)
   {
      return jointActuationByName.get(jointName);
   }

   /** Every joint that has a low-level command block, in model order. */
   public List<MujocoJointActuation> getJointActuations()
   {
      return jointActuationList;
   }

   /**
    * Write each joint's combined setpoint into {@code mjData.ctrl} and, when they have changed, its
    * gains into {@code mjModel.actuator_biasprm}.
    *
    * <p>A single affine actuator carries the whole law: {@code gainprm[0]} stays at the 1 the MJCF
    * gave it, {@code biasprm} holds {@code 0, -kp, -kd}, and {@code ctrl} carries
    * {@code tau_ff + kp * q_d + kd * qd_d}. Writing gains into {@code mjModel} between steps is
    * supported -- they take part in no derived constant and change no array size, so no recompile is
    * needed; they live in the model rather than in {@code mjData} purely because MuJoCo treats them
    * as actuator description.
    *
    * <p>The joint state is captured here too. This runs after the state push, so the mecano joints
    * and {@code mjData} agree, and it is exactly the state MuJoCo will evaluate the actuator at --
    * which is what lets the torque decomposition be computed rather than guessed.
    *
    * <p>The command comes from {@code ControllerOutput}, the same place the effort does, so a
    * controller reaches the actuators without knowing MuJoCo exists. A controller that publishes
    * only an effort -- which is all SCS2's engine contract has ever required -- gets it applied
    * through the actuator with zero gains, which is the same force an applied torque would have
    * been.
    */
   public void pushActuationToMujoco(mjModel model, mjData data)
   {
      if (jointActuationList.isEmpty())
         return;

      DoublePointer ctrl = data.ctrl();
      DoublePointer biasprm = model.actuator_biasprm();

      for (int i = 0; i < jointActuationList.size(); i++)
      {
         MujocoJointActuation actuation = jointActuationList.get(i);
         int index = actuation.getActuatorIndex();

         OneDoFJointBasics joint = actuatedJoints.get(i);
         OneDoFJointStateReadOnly jointOutput = actuatedJointOutputs.get(i);
         // Polled unconditionally so a directly staged command never carries into the next tick.
         boolean stagedDirectly = actuation.pollCommanded();

         if (jointOutput != null && jointOutput.hasCommand())
         {
            // In a controller's output the configuration and velocity are setpoints, not measurements.
            actuation.setCommandFromControllerOutput(jointOutput.getFeedforwardEffort(),
                                                     jointOutput.getConfiguration(),
                                                     jointOutput.getVelocity(),
                                                     jointOutput.getStiffness(),
                                                     jointOutput.getDamping());
         }
         else if (!stagedDirectly)
         {
            // No command from the controller and nobody staged one directly, so honor SCS2's own
            // contract and use the effort that was written. Zero gains make the actuator produce
            // exactly that torque, which is what an applied force would have done.
            actuation.setFeedforwardOnly(joint.getTau());
         }
         actuation.setStateAtCommand(joint.getQ(), joint.getQd());

         ctrl.put(index, actuation.getCombinedControl());

         if (actuation.gainsNeedWriting())
         {
            biasprm.put((long) index * MJ_NBIAS + 1, -actuation.getStiffness());
            biasprm.put((long) index * MJ_NBIAS + 2, -actuation.getDamping());
            actuation.markGainsWritten();
         }
      }
   }

   /**
    * Split each joint's applied torque into its three terms and write the total onto the SCS2 joint,
    * so {@code tau} reflects what MuJoCo did rather than the stale value the controller wrote.
    */
   public void pullActuationFromMujoco(mjData data)
   {
      if (jointActuationList.isEmpty())
         return;

      DoublePointer actuatorForce = data.actuator_force();
      DoublePointer qfrcActuator = data.qfrc_actuator();

      for (int i = 0; i < jointActuationList.size(); i++)
      {
         MujocoJointActuation actuation = jointActuationList.get(i);
         actuation.updateTorqueDecomposition(actuatorForce.get(actuation.getActuatorIndex()));

         int dofAddress = actuatedJointDofAddresses[i];
         if (dofAddress >= 0)
            actuatedJoints.get(i).setTau(qfrcActuator.get(dofAddress));
      }
   }

   /** Joints whose pin was already engaged last tick, so the latched target must be left alone. */
   private final Set<String> pinnedLastTick = new HashSet<>();
   private final Quaternion pinOrientation = new Quaternion();
   private final Vector3D pinPosition = new Vector3D();

   // gainprm is not written at runtime: the single servo actuator keeps the gain of 1 the MJCF gave
   // it, and the gains ride in biasprm.
   private static final int MJ_NBIAS = Mujoco.mjNBIAS;

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
    * Write the SCS2 joint state ({@code q}, {@code qd}) into MuJoCo's {@code qpos} / {@code qvel} for each managed
    * joint. Torque does not travel this way: every joint with a degree of freedom has an actuator, and
    * {@code pushActuationToMujoco} drives it.
    *
    * <p>The SCS2 joints are the state of record, as with the other SCS2 engines: anything that edits them between
    * steps (a test teleporting the robot, a GUI edit, rewinding the buffer and resuming) takes effect on the next
    * step. A joint's state is only written when it differs from MuJoCo's by more than {@link #STATE_EDIT_EPSILON}:
    * the pull/push round trip (frame conversion of the root velocity, quaternion handling) leaves ~1e-16 of
    * floating-point noise, and writing that back every step would perturb an untouched simulation. No
    * {@code mj_forward} is needed: {@code mj_step} recomputes everything from {@code qpos} / {@code qvel}.
    */
   public void pushStateToMujoco(DoublePointer qpos, DoublePointer qvel)
   {
      for (JointBasics joint : getJointsToConsider())
      {
         JointAddress address = mujocoMultiBodyRobot.getJointAddress(joint.getName());
         if (address == null)
            continue;
         if (address.isFloatingRoot && joint instanceof SixDoFJointBasics floating)
         {
            int qp = address.qposadr;
            int qv = address.qveladr;
            // MuJoCo quaternion order is (w, x, y, z).
            quaternion.set(floating.getJointPose().getOrientation());
            // Inverse of pullStateFromMujoco: the freejoint's linear velocity is in the world frame and its angular
            // velocity in the body frame, while the mecano twist has both in the body frame.
            linearVelocity.set(floating.getJointTwist().getLinearPart());
            quaternion.transform(linearVelocity);
            angularVelocity.set(floating.getJointTwist().getAngularPart());

            boolean edited = isEdited(qpos, qp, floating.getJointPose().getPosition()) || !isSameOrientation(qpos, qp + 3, quaternion)
                             || isEdited(qvel, qv, linearVelocity) || isEdited(qvel, qv + 3, angularVelocity);
            if (edited)
            {
               qpos.put(qp, floating.getJointPose().getX());
               qpos.put(qp + 1, floating.getJointPose().getY());
               qpos.put(qp + 2, floating.getJointPose().getZ());
               qpos.put(qp + 3, quaternion.getS());
               qpos.put(qp + 4, quaternion.getX());
               qpos.put(qp + 5, quaternion.getY());
               qpos.put(qp + 6, quaternion.getZ());
               qvel.put(qv, linearVelocity.getX());
               qvel.put(qv + 1, linearVelocity.getY());
               qvel.put(qv + 2, linearVelocity.getZ());
               qvel.put(qv + 3, angularVelocity.getX());
               qvel.put(qv + 4, angularVelocity.getY());
               qvel.put(qv + 5, angularVelocity.getZ());
            }
         }
         else if (joint instanceof OneDoFJointBasics oneDoF)
         {
            if (Math.abs(qpos.get(address.qposadr) - oneDoF.getQ()) > STATE_EDIT_EPSILON)
               qpos.put(address.qposadr, oneDoF.getQ());
            if (Math.abs(qvel.get(address.qveladr) - oneDoF.getQd()) > STATE_EDIT_EPSILON)
               qvel.put(address.qveladr, oneDoF.getQd());
            // Every 1-DoF joint with a degree of freedom in the model also has an actuator: the
            // MJCF builder emits one for each, and refuses joint types it cannot represent rather
            // than emitting a jointless body. So the torque always arrives through the actuator,
            // and there is no second path to keep in step with it.
            if (!jointActuationByName.containsKey(joint.getName()))
            {
               throw new IllegalStateException("Joint '" + joint.getName()
                                               + "' has a degree of freedom in the MuJoCo model but no actuator to drive it, so its torque"
                                               + " would be silently dropped. This is a bug in the MJCF builder.");
            }
         }
      }
   }

   /**
    * Mirror each joint's {@code isPinned()} flag onto its MuJoCo equality constraint, holding the
    * joint at the state SCS2 currently has for it.
    *
    * <p>Pinning is an existing SCS2 concept ({@link SimJointBasics#setPinned(boolean)}), exposed as
    * a live YoBoolean per joint, and honored by ContactPointBased and ImpulseBased. Those engines
    * freeze the joint outside the solver; here the constraint goes through the same solver that
    * resolves contact, so a pinned body pushes back on whatever touches it instead of being
    * silently teleported after the step.
    *
    * <p>Combined with {@code pushStateToMujoco}, the usual "move it, then hold it" recipe works:
    * set the joint state through SCS2 and set pinned, and MuJoCo holds it there.
    */
   public void pushPinnedJointsToMujoco(mjModel model, mjData data)
   {
      DoublePointer eqData = model.eq_data();
      DoublePointer qpos0 = model.qpos0();
      DoublePointer qpos = data.qpos();
      BoolPointer eqActive = data.eq_active();

      for (SimJointBasics joint : getJointsToConsider())
      {
         int equalityId = mujocoMultiBodyRobot.getPinEqualityId(joint.getName());
         if (equalityId < 0)
            continue;

         boolean pinned = joint.isPinned();
         eqActive.put(equalityId, pinned);
         if (!pinned)
         {
            pinnedLastTick.remove(joint.getName());
            continue;
         }

         JointAddress address = mujocoMultiBodyRobot.getJointAddress(joint.getName());
         if (address == null)
            continue;

         // The target is latched rather than refreshed from the joint every tick. While pinned the
         // SCS2 joint mirrors MuJoCo, so re-deriving the target from it would chase the constraint's
         // own residual and walk the pin away from where it was set. Re-latch on two events only:
         // the tick pinning is switched on, and any tick where the SCS2 state has been edited away
         // from MuJoCo's (which is why this runs before pushStateToMujoco writes such an edit in).
         boolean justPinned = pinnedLastTick.add(joint.getName());
         if (!justPinned && !isJointStateEdited(joint, address, qpos))
            continue;

         int base = equalityId * MJ_NEQDATA;
         if (address.isFloatingRoot && joint instanceof SixDoFJointBasics floating)
         {
            // Weld layout: [anchor(3), relpose position(3), relpose quaternion(4), torquescale(1)].
            // relpose is the pose of body2 in body1, and body2 here is the world, so holding the
            // root at a world pose means writing that pose's inverse.
            pinOrientation.setAndConjugate(floating.getJointPose().getOrientation());
            pinPosition.setAndNegate(floating.getJointPose().getPosition());
            pinOrientation.transform(pinPosition);

            eqData.put(base + 3, pinPosition.getX());
            eqData.put(base + 4, pinPosition.getY());
            eqData.put(base + 5, pinPosition.getZ());
            eqData.put(base + 6, pinOrientation.getS());
            eqData.put(base + 7, pinOrientation.getX());
            eqData.put(base + 8, pinOrientation.getY());
            eqData.put(base + 9, pinOrientation.getZ());
         }
         else if (joint instanceof OneDoFJointBasics oneDoF)
         {
            // Joint-equality layout with joint2 omitted: y = y0 + data[0], where y0 is the joint's
            // value at the model's default pose.
            eqData.put(base, oneDoF.getQ() - qpos0.get(address.qposadr));
         }
      }
   }

   /** True when the SCS2 joint has been moved away from what MuJoCo currently holds. */
   private boolean isJointStateEdited(SimJointBasics joint, JointAddress address, DoublePointer qpos)
   {
      if (address.isFloatingRoot && joint instanceof SixDoFJointBasics floating)
      {
         pinOrientation.set(floating.getJointPose().getOrientation());
         return isEdited(qpos, address.qposadr, floating.getJointPose().getPosition())
                || !isSameOrientation(qpos, address.qposadr + 3, pinOrientation);
      }
      if (joint instanceof OneDoFJointBasics oneDoF)
         return Math.abs(qpos.get(address.qposadr) - oneDoF.getQ()) > STATE_EDIT_EPSILON;
      return false;
   }

   /** Size of one {@code mjModel.eq_data} row; mirrors MuJoCo's {@code mjNEQDATA}. */
   private static final int MJ_NEQDATA = Mujoco.mjNEQDATA;

   /** Differences below this are floating-point noise from the pull/push round trip, not edits made through SCS2. */
   private static final double STATE_EDIT_EPSILON = 1.0e-10;

   private static boolean isEdited(DoublePointer array, int start, us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly value)
   {
      return Math.abs(array.get(start) - value.getX()) > STATE_EDIT_EPSILON || Math.abs(array.get(start + 1) - value.getY()) > STATE_EDIT_EPSILON
             || Math.abs(array.get(start + 2) - value.getZ()) > STATE_EDIT_EPSILON;
   }

   /** Compares a MuJoCo (w, x, y, z) quaternion with {@code orientation}, treating q and -q as the same rotation. */
   private static boolean isSameOrientation(DoublePointer array, int start, Quaternion orientation)
   {
      double w = array.get(start), x = array.get(start + 1), y = array.get(start + 2), z = array.get(start + 3);
      boolean same = Math.abs(w - orientation.getS()) <= STATE_EDIT_EPSILON && Math.abs(x - orientation.getX()) <= STATE_EDIT_EPSILON
                     && Math.abs(y - orientation.getY()) <= STATE_EDIT_EPSILON && Math.abs(z - orientation.getZ()) <= STATE_EDIT_EPSILON;
      boolean opposite = Math.abs(w + orientation.getS()) <= STATE_EDIT_EPSILON && Math.abs(x + orientation.getX()) <= STATE_EDIT_EPSILON
                         && Math.abs(y + orientation.getY()) <= STATE_EDIT_EPSILON && Math.abs(z + orientation.getZ()) <= STATE_EDIT_EPSILON;
      return same || opposite;
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
      // The session owns gravity and can change it between ticks, so the IMU's acceleration
      // calculator has to track it rather than assume standard gravity.
      accelerationCalculator.setGravitionalAcceleration(gravity);

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
