package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateReadOnly;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoMultiBodyRobot.JointAddress;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoActuationMode;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoContactProperties;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Generates the composite MJCF text for {@link MujocoMultiBodyDynamicsWorld#compile} and, after
 * compile, walks each robot's {@link RobotDefinition} to register joint addresses on the
 * matching {@link MujocoMultiBodyRobot}.
 *
 * <p>v1 strategy: emit MJCF directly from {@link RobotDefinition} and
 * {@link TerrainObjectDefinition}. The earlier URDF-include design didn't work in practice --
 * MuJoCo's {@code <include>} expects MJCF, not URDF. Generating MJCF here keeps every MuJoCo
 * physics knob (contact, solver, options) on the table and avoids a fragile XML round-trip.
 *
 * <p>v1 limitations called out explicitly:
 * <ul>
 *   <li>Joint coverage: {@link SixDoFJointDefinition} (root freejoint), revolute, prismatic. Other
 *       joint types (planar, spherical, cross-four-bar, fixed) throw or are silently skipped.</li>
 *   <li>Geometry coverage: box, sphere, capsule, cylinder. Meshes / convex polytopes require
 *       MuJoCo {@code <asset>} entries and are not yet handled.</li>
 *   <li>Inertia is emitted via {@code fullinertia="ixx iyy izz ixy ixz iyz"} so non-diagonal
 *       moment tensors are supported, but the inertia frame is assumed aligned with the body
 *       frame (i.e. the SCS2 {@code RigidBodyDefinition.getInertiaPose()} rotation is ignored;
 *       MuJoCo doesn't expose an inertia frame attribute and treats inertia as in the body
 *       frame).</li>
 * </ul>
 */
public final class MujocoMultiBodyRobotFactory
{
   // Collision groups: the coarse foot box would self-collide (swing foot catching the stance
   // foot) in ways the real robot doesn't, so contype/conaffinity restrict robot geoms to test
   // only against terrain (robot=1/2, terrain=2/1), mirroring ContactPointBased.
   private static final int ROBOT_CONTYPE = 1;
   private static final int ROBOT_CONAFFINITY = 2;
   private static final int TERRAIN_CONTYPE = 2;
   private static final int TERRAIN_CONAFFINITY = 1;

   /** Name of the keyframe holding the world's initial state; index 0, the only keyframe emitted. */
   public static final String INITIAL_KEYFRAME_NAME = "initial";

   /** Appended to a joint name to form the name of its pin equality constraint in the MJCF. */
   public static final String PIN_EQUALITY_SUFFIX = "_pin";
   /**
    * Solver settings for the pin equalities. Pinning in SCS2 means "hold this joint exactly", so
    * these are much stiffer than MuJoCo's equality defaults (solref 0.02, solimp dmax 0.95), which
    * would let a pinned pelvis visibly sag under the robot's own weight. The 0.005 s time constant
    * stays above the usual 2 * timestep stability floor for the session rates in use.
    */
   private static final String PIN_EQUALITY_SOLVER_ATTRIBUTES = " solref=\"0.005 1\" solimp=\"0.95 0.9999 0.001\"";

   /** Name suffixes of a joint's three JOINT_SERVO actuators, in the order they are emitted. */
   public static final String ACTUATOR_SUFFIX_CONTROLLER_TAU = "_ControllerTau";
   public static final String ACTUATOR_SUFFIX_POSITION_TAU = "_PositionTau";
   public static final String ACTUATOR_SUFFIX_VELOCITY_TAU = "_VelocityTau";

   private MujocoMultiBodyRobotFactory()
   {
   }

   /**
    * Build the composite MJCF text for the entire world.
    */
   public static String buildWorldMjcf(List<Robot> robots,
                                       List<TerrainObjectDefinition> terrainObjects,
                                       File workingDirectory,
                                       MujocoSimulationParametersReadOnly parameters)
   {
      StringBuilder mjcf = new StringBuilder();
      mjcf.append("<mujoco>\n");
      // mjOption values (solver, integrator, tolerances, ...) are deliberately not emitted: the
      // engine pushes the full YoMujocoOptions group into the compiled model before the first step,
      // and the session pushes gravity every tick. Only the structural flag lives in the MJCF.
      mjcf.append("  <option>\n");
      mjcf.append("    <flag filterparent=\"").append(parameters.getFilterParentCollisions() ? "enable" : "disable").append("\"/>\n");
      mjcf.append("  </option>\n");
      // meshdir is absolute because the MJCF is compiled out of a virtual file system and so has no
      // directory of its own for MuJoCo to resolve mesh filenames against. Harmless when the world
      // has no file-backed meshes, in which case the directory is never even created.
      mjcf.append("  <compiler angle=\"radian\" meshdir=\"").append(workingDirectory.getAbsolutePath()).append("\"/>\n");
      // Compile-time seeds; live-tunable after compile via the MujocoOptions o_* contact override.
      mjcf.append("  <default>\n");
      mjcf.append("    <geom friction=\"").append(parameters.get_friction_slide())
          .append(' ').append(parameters.get_friction_spin())
          .append(' ').append(parameters.get_friction_roll())
          .append("\" solref=\"").append(parameters.get_solref_timeconst()).append(' ').append(parameters.get_solref_dampratio())
          .append("\" solimp=\"").append(parameters.get_solimp_dmin())
          .append(' ').append(parameters.get_solimp_dmax())
          .append(' ').append(parameters.get_solimp_width())
          .append(' ').append(parameters.get_solimp_midpoint())
          .append(' ').append(parameters.get_solimp_power())
          .append("\" condim=\"").append(parameters.get_condim())
          .append("\" margin=\"").append(parameters.get_margin())
          .append("\" gap=\"").append(parameters.get_gap())
          .append("\"/>\n");
      mjcf.append("    <joint armature=\"").append(parameters.get_armature()).append("\"/>\n");
      mjcf.append("    <default class=\"robot\">\n");
      mjcf.append("      <geom contype=\"").append(ROBOT_CONTYPE).append("\" conaffinity=\"").append(ROBOT_CONAFFINITY).append("\"/>\n");
      // Contact classes nest inside the robot class, so a foot or hand class inherits the robot's
      // contype/conaffinity and overrides only the contact properties it sets.
      for (Map.Entry<String, MujocoContactProperties> contactClass : parameters.getContactClasses().entrySet())
      {
         StringBuilder attributes = new StringBuilder();
         if (!contactClass.getValue().appendGeomAttributes(attributes))
            continue;
         mjcf.append("      <default class=\"").append(contactClass.getKey()).append("\">\n");
         mjcf.append("        <geom").append(attributes).append("/>\n");
         mjcf.append("      </default>\n");
      }
      mjcf.append("    </default>\n");
      mjcf.append("    <default class=\"terrain\">\n");
      mjcf.append("      <geom contype=\"").append(TERRAIN_CONTYPE).append("\" conaffinity=\"").append(TERRAIN_CONAFFINITY).append("\"/>\n");
      mjcf.append("    </default>\n");
      mjcf.append("  </default>\n");
      // <asset> (mesh terrain) must precede <worldbody>. Only emitted when some terrain shape needs a
      // mesh (convex polytope / ramp); primitive-only worlds produce an empty fragment and no block.
      StringBuilder meshAssets = new StringBuilder();
      for (int i = 0; i < terrainObjects.size(); i++)
      {
         meshAssets.append(MujocoTerrainFactory.toMjcfAssetFragment(terrainObjects.get(i), "terrain_" + i + "_"));
      }
      for (Robot robot : robots)
      {
         appendRobotMeshAssets(meshAssets, robot.getRobotDefinition(), workingDirectory);
      }
      if (meshAssets.length() > 0)
      {
         mjcf.append("  <asset>\n").append(meshAssets).append("  </asset>\n");
      }
      mjcf.append("  <worldbody>\n");
      for (int i = 0; i < terrainObjects.size(); i++)
      {
         mjcf.append(MujocoTerrainFactory.toMjcfWorldbodyFragment(terrainObjects.get(i), "terrain_" + i + "_"));
      }
      for (Robot robot : robots)
      {
         RobotDefinition robotDefinition = robot.getRobotDefinition();
         Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());
         appendRobotBodies(mjcf, robotDefinition, ignoredJointNames, parameters, 2);
      }
      mjcf.append("  </worldbody>\n");
      StringBuilder pinEqualities = new StringBuilder();
      for (Robot robot : robots)
      {
         appendPinEqualities(pinEqualities, robot.getRobotDefinition(), 2);
      }
      if (pinEqualities.length() > 0)
      {
         mjcf.append("  <equality>\n").append(pinEqualities).append("  </equality>\n");
      }
      if (parameters.getFilterParentCollisions())
      {
         for (Robot robot : robots)
         {
            appendParentChildContactExcludes(mjcf, robot.getRobotDefinition(), 2);
         }
      }
      if (parameters.getActuationMode() == MujocoActuationMode.JOINT_SERVO)
      {
         StringBuilder actuators = new StringBuilder();
         for (Robot robot : robots)
         {
            appendJointServoActuators(actuators, robot.getRobotDefinition(), parameters, 2);
         }
         if (actuators.length() > 0)
         {
            mjcf.append("  <actuator>\n").append(actuators).append("  </actuator>\n");
         }
      }
      // One empty keyframe, which MuJoCo fills with qpos0. The engine overwrites it with the seeded
      // initial state once the model has compiled -- writing the vector into the XML here is not
      // possible, since the qpos layout is only known after compile.
      mjcf.append("  <keyframe>\n    <key name=\"").append(INITIAL_KEYFRAME_NAME).append("\"/>\n  </keyframe>\n");
      mjcf.append("</mujoco>\n");
      return mjcf.toString();
   }

   /**
    * Register every joint in {@code robotDefinition} on the supplied {@link MujocoMultiBodyRobot}.
    * The world must already be compiled (model != null) before calling. Joints listed in
    * {@code robotDefinition.getNameOfJointsToIgnore()} are skipped: they don't appear in the MJCF
    * (see {@link #appendBody}) and the controller never commands them, so there's nothing to map.
    */
   public static MujocoMultiBodyRobot registerJoints(RobotDefinition robotDefinition, mjModel model)
   {
      MujocoMultiBodyRobot mujocoRobot = new MujocoMultiBodyRobot(robotDefinition.getName(), model);
      Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         // A welded joint has no <joint> element to resolve, but its body does exist in the MJCF
         // and still needs a body id so contact wrenches and external wrenches can be routed to it.
         if (!isWeldedToParent(jointDefinition, ignoredJointNames))
         {
            boolean isFloatingRoot = jointDefinition instanceof SixDoFJointDefinition
                                     && jointDefinition.getParentJoint() == null;
            try
            {
               mujocoRobot.registerJoint(jointDefinition.getName(), isFloatingRoot);
            }
            catch (RuntimeException e)
            {
               System.err.println("[MujocoMultiBodyRobotFactory] SKIPPED joint '" + jointDefinition.getName() + "': " + e.getMessage());
            }
            mujocoRobot.registerPinEquality(jointDefinition.getName());
            mujocoRobot.registerJointServoActuators(jointDefinition.getName());
         }
         RigidBodyDefinition successor = jointDefinition.getSuccessor();
         if (successor != null)
            mujocoRobot.registerBody(successor.getName());
      }
      return mujocoRobot;
   }

   /**
    * Push the {@code q} component of each non-root joint's {@code OneDoFJointStateReadOnly} initial
    * state into {@code mjData.qpos}. Without this step Alex (and any humanoid spawned via
    * {@code HumanoidRobotInitialSetup.initializeRobotDefinition}) starts at all-zero joint angles
    * (arms straight down, knees locked) and immediately collapses, even though the
    * RobotDefinition carries a perfectly good half-squat pose. The root SixDoF freejoint is
    * already seeded by the body's {@code pos}/{@code quat} attributes emitted in MJCF, so we leave
    * it alone here.
    */
   public static void seedInitialJointState(RobotDefinition robotDefinition, MujocoMultiBodyRobot mujocoRobot, mjData data)
   {
      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         if (!(jointDefinition instanceof OneDoFJointDefinition))
            continue;
         JointAddress address = mujocoRobot.getJointAddress(jointDefinition.getName());
         if (address == null || address.isFloatingRoot)
            continue;
         if (!(jointDefinition.getInitialJointState() instanceof OneDoFJointStateReadOnly initial))
            continue;
         double q = initial.getConfiguration();
         if (!Double.isNaN(q))
            data.qpos().put(address.qposadr, q);
         double qd = initial.getVelocity();
         if (!Double.isNaN(qd))
            data.qvel().put(address.qveladr, qd);
      }
   }

   private static void appendRobotBodies(StringBuilder sb,
                                         RobotDefinition robotDefinition,
                                         Set<String> ignoredJointNames,
                                         MujocoSimulationParametersReadOnly parameters,
                                         int indentLevel)
   {
      String namePrefix = robotDefinition.getName() + "_";
      List<JointDefinition> rootJoints = robotDefinition.getRootJointDefinitions();
      for (JointDefinition rootJoint : rootJoints)
      {
         appendBody(sb, rootJoint, rootJoint.getSuccessor(), namePrefix, ignoredJointNames, parameters, false, indentLevel);
      }
   }

   /**
    * Emit one {@code <body>} and recurse into its children.
    *
    * <p>{@code weldToParent} suppresses the {@code <joint>} element, which is how MuJoCo expresses
    * a body rigidly attached to its parent. That is what a joint in
    * {@link RobotDefinition#getNameOfJointsToIgnore()} means for the dynamics: mecano's
    * {@code ForwardDynamicsCalculator} defaults {@code considerIgnoredSubtreesInertia} to true, so
    * the other engines keep the subtree's inertia rigidly attached. Dropping those bodies from the
    * MJCF entirely, as this used to, silently lost their mass under MuJoCo only -- which on the
    * hand-equipped Alex versions is the whole hand.
    */
   private static void appendBody(StringBuilder sb,
                                  JointDefinition joint,
                                  RigidBodyDefinition body,
                                  String namePrefix,
                                  Set<String> ignoredJointNames,
                                  MujocoSimulationParametersReadOnly parameters,
                                  boolean weldToParent,
                                  int indent)
   {
      String pad = "  ".repeat(indent);

      sb.append(pad).append("<body name=\"").append(namePrefix).append(body.getName()).append('"');
      // For the root joint, place the body at its initial pose (MuJoCo uses the body's pos/quat
      // attributes as the starting qpos for the freejoint). Non-root joints use transformToParent
      // since the parent body's frame is the reference. A welded joint additionally folds in its
      // initial configuration, since it has no qpos entry to carry it.
      RigidBodyTransform spawnTransform = computeSpawnTransform(joint);
      if (weldToParent)
         appendWeldedJointConfiguration(spawnTransform, joint);
      if (!MujocoTools.isIdentity(spawnTransform))
         sb.append(' ').append(MujocoTools.toPosQuatAttributes(spawnTransform));
      sb.append(">\n");

      if (!weldToParent)
         MujocoTools.appendJoint(sb, joint, namePrefix, parameters, indent + 1);
      MujocoTools.appendInertial(sb, body, indent + 1);

      // A body assigned a contact class uses it in place of the plain robot class; the class is
      // nested inside "robot", so the collision filtering is inherited either way.
      String geomClass = parameters.getContactClassByBodyName().getOrDefault(body.getName(), "robot");
      int geomIndex = 0;
      for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
      {
         MujocoTools.appendGeom(sb, geomClass, namePrefix + body.getName() + "_geom_" + geomIndex, shape, indent + 1);
         geomIndex++;
      }

      for (JointDefinition childJoint : body.getChildrenJoints())
      {
         if (childJoint.getSuccessor() == null)
            continue;
         boolean childWelded = weldToParent || ignoredJointNames.contains(childJoint.getName());
         appendBody(sb, childJoint, childJoint.getSuccessor(), namePrefix, ignoredJointNames, parameters, childWelded, indent + 1);
      }

      sb.append(pad).append("</body>\n");
   }

   /**
    * Fold a welded 1-DoF joint's initial configuration into its fixed transform, so a subtree
    * ignored at a non-zero angle is welded in the pose it actually holds rather than at q = 0.
    */
   private static void appendWeldedJointConfiguration(RigidBodyTransform transformToUpdate, JointDefinition joint)
   {
      if (!(joint instanceof OneDoFJointDefinition oneDoFJoint))
         return;
      if (!(joint.getInitialJointState() instanceof OneDoFJointStateReadOnly initialState))
         return;
      double q = initialState.getConfiguration();
      if (Double.isNaN(q) || q == 0.0)
         return;

      Vector3D axis = new Vector3D(oneDoFJoint.getAxis());
      RigidBodyTransform jointTransform = new RigidBodyTransform();
      if (joint instanceof RevoluteJointDefinition)
      {
         axis.normalize();
         axis.scale(q);
         jointTransform.getRotation().setRotationVector(axis);
      }
      else if (joint instanceof PrismaticJointDefinition)
      {
         axis.normalize();
         axis.scale(q);
         jointTransform.getTranslation().set(axis);
      }
      else
      {
         return;
      }
      transformToUpdate.multiply(jointTransform);
   }

   private static void appendRobotMeshAssets(StringBuilder sb, RobotDefinition robotDefinition, File workingDirectory)
   {
      String namePrefix = robotDefinition.getName() + "_";
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         int geomIndex = 0;
         for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
         {
            MujocoTools.appendMeshAsset(sb, namePrefix + body.getName() + "_geom_" + geomIndex, shape, 2, workingDirectory);
            geomIndex++;
         }
      }
   }

   /**
    * Emit one disabled equality constraint per joint, used to implement
    * {@code SimJointBasics.setPinned(boolean)}.
    *
    * <p>The other SCS2 engines pin a joint by switching it to an acceleration source and skipping
    * its integration, which freezes it without the solver knowing. MuJoCo can do better: an
    * equality constraint holds the joint through the same solver that resolves contact, so a
    * pinned body produces correct reaction forces instead of being teleported back after the step.
    *
    * <p>A free root joint gets a {@code weld} to the world; a 1-DoF joint gets a {@code joint}
    * equality with no second joint, which MuJoCo constrains to a constant. All are emitted
    * inactive; {@code MujocoRobot.pushPinnedJointsToMujoco} flips {@code mjData.eq_active} and
    * writes the held target into {@code mjModel.eq_data} each tick.
    */
   private static void appendPinEqualities(StringBuilder sb, RobotDefinition robotDefinition, int indent)
   {
      String namePrefix = robotDefinition.getName() + "_";
      String pad = "  ".repeat(indent);
      Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());

      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (isWeldedToParent(joint, ignoredJointNames))
            continue;

         String equalityName = namePrefix + joint.getName() + PIN_EQUALITY_SUFFIX;
         if (joint instanceof SixDoFJointDefinition && joint.getParentJoint() == null)
         {
            RigidBodyDefinition rootBody = joint.getSuccessor();
            if (rootBody == null)
               continue;
            // body2 defaults to the world, so this welds the root body to a world-frame pose.
            sb.append(pad).append("<weld name=\"").append(equalityName)
              .append("\" body1=\"").append(namePrefix).append(rootBody.getName())
              .append("\" active=\"false\"").append(PIN_EQUALITY_SOLVER_ATTRIBUTES).append("/>\n");
         }
         else if (joint instanceof OneDoFJointDefinition)
         {
            sb.append(pad).append("<joint name=\"").append(equalityName)
              .append("\" joint1=\"").append(namePrefix).append(joint.getName())
              .append("\" active=\"false\"").append(PIN_EQUALITY_SOLVER_ATTRIBUTES).append("/>\n");
         }
      }
   }

   /**
    * Emit the three actuators per 1-DoF joint that make up
    * {@link MujocoActuationMode#JOINT_SERVO}, in a fixed order so a joint's actuators are always
    * {@code base + 0, 1, 2}.
    *
    * <p>With {@code mjBIAS_AFFINE} an actuator's force is
    * {@code gainprm[0] * ctrl + biasprm[0] + biasprm[1] * q + biasprm[2] * qdot}, so the three
    * together reproduce {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} once
    * {@code MujocoRobot.pushActuationToMujoco} fills in the gains and setpoints:
    *
    * <pre>
    *   ControllerTau  ctrl = tau_ff  gainprm[0] = 1   biasprm = 0, 0,   0
    *   PositionTau    ctrl = q_d     gainprm[0] = kp  biasprm = 0, -kp, 0
    *   VelocityTau    ctrl = qd_d    gainprm[0] = kd  biasprm = 0, 0,   -kd
    * </pre>
    *
    * <p>Splitting the law across three actuators rather than folding it into one is what keeps the
    * controller/position/velocity decomposition that {@code SCS2OutputWriter} publishes: MuJoCo
    * reports each actuator's force separately in {@code mjData.actuator_force}.
    *
    * <p>{@code <general>} is used rather than the {@code <position>}/{@code <velocity>} shortcuts
    * because those bake their gains into the compiled model and set a {@code ctrlrange}; the gains
    * here have to be writable every tick, since the controller re-sends them every tick.
    */
   private static void appendJointServoActuators(StringBuilder sb,
                                                RobotDefinition robotDefinition,
                                                MujocoSimulationParametersReadOnly parameters,
                                                int indent)
   {
      String namePrefix = robotDefinition.getName() + "_";
      String pad = "  ".repeat(indent);
      Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());

      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (!(joint instanceof OneDoFJointDefinition))
            continue;
         if (isWeldedToParent(joint, ignoredJointNames))
            continue;

         String jointName = namePrefix + joint.getName();
         // The delay goes on all three actuators: they are three halves of one drive's command, so
         // the whole command has to arrive late together. MuJoCo rejects a delay without nsample,
         // and linear interpolation avoids the staircase a nearest-sample lookup would produce.
         double delay = parameters.getActuatorDelayByJointName().getOrDefault(joint.getName(), parameters.getActuatorDelay());
         String delayAttributes = delay > 0.0 ? " delay=\"" + delay + "\" nsample=\"" + parameters.getActuatorDelaySamples() + "\" interp=\"linear\"" : "";

         appendGeneralActuator(sb, pad, jointName + ACTUATOR_SUFFIX_CONTROLLER_TAU, jointName, "1 0 0", null, delayAttributes);
         appendGeneralActuator(sb, pad, jointName + ACTUATOR_SUFFIX_POSITION_TAU, jointName, "0 0 0", "0 0 0", delayAttributes);
         appendGeneralActuator(sb, pad, jointName + ACTUATOR_SUFFIX_VELOCITY_TAU, jointName, "0 0 0", "0 0 0", delayAttributes);
      }
   }

   private static void appendGeneralActuator(StringBuilder sb,
                                            String pad,
                                            String name,
                                            String jointName,
                                            String gainprm,
                                            String biasprm,
                                            String extraAttributes)
   {
      sb.append(pad).append("<general name=\"").append(name)
        .append("\" joint=\"").append(jointName)
        .append("\" ctrllimited=\"false\" gaintype=\"fixed\" gainprm=\"").append(gainprm).append('"');
      if (biasprm == null)
         sb.append(" biastype=\"none\"");
      else
         sb.append(" biastype=\"affine\" biasprm=\"").append(biasprm).append('"');
      sb.append(extraAttributes);
      sb.append("/>\n");
   }

   private static void appendParentChildContactExcludes(StringBuilder sb, RobotDefinition robotDefinition, int indent)
   {
      String namePrefix = robotDefinition.getName() + "_";
      String pad = "  ".repeat(indent);
      StringBuilder excludes = new StringBuilder();
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         JointDefinition parentJoint = joint.getParentJoint();
         RigidBodyDefinition childBody = joint.getSuccessor();
         if (parentJoint == null || childBody == null)
            continue;

         RigidBodyDefinition parentBody = parentJoint.getSuccessor();
         if (parentBody == null)
            continue;

         excludes.append(pad).append("  <exclude body1=\"").append(namePrefix).append(parentBody.getName())
                 .append("\" body2=\"").append(namePrefix).append(childBody.getName()).append("\"/>\n");
      }

      if (excludes.length() == 0)
         return;

      sb.append(pad).append("<contact>\n").append(excludes).append(pad).append("</contact>\n");
   }

   /**
    * True when this joint (or an ancestor) is in {@code ignoredJointNames}, i.e. it is welded to its
    * parent in the MJCF and so has no {@code <joint>} element to resolve. Matches {@link #appendBody}.
    */
   private static boolean isWeldedToParent(JointDefinition joint, Set<String> ignoredJointNames)
   {
      for (JointDefinition current = joint; current != null; current = current.getParentJoint())
      {
         if (ignoredJointNames.contains(current.getName()))
            return true;
      }
      return false;
   }

   private static RigidBodyTransform computeSpawnTransform(JointDefinition joint)
   {
      // Root joint: spawn pose comes from initial joint state (SCS2 sets this on the joint def).
      // Non-root joint: spawn pose is the transform from parent body to this joint.
      RigidBodyTransform transform = new RigidBodyTransform();
      if (joint.getParentJoint() == null && joint instanceof SixDoFJointDefinition)
      {
         if (joint.getInitialJointState() instanceof SixDoFJointState initial)
         {
            Quaternion orientation = new Quaternion();
            orientation.set(initial.getOrientation());
            transform.set(orientation, initial.getPosition());
         }
      }
      else if (joint.getTransformToParent() != null)
      {
         transform.set(joint.getTransformToParent());
      }
      return transform;
   }
}
