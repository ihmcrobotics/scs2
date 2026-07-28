package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
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
      if (!workingDirectory.exists() && !workingDirectory.mkdirs())
         throw new RuntimeException("Could not create MuJoCo working directory: " + workingDirectory);

      StringBuilder mjcf = new StringBuilder();
      mjcf.append("<mujoco>\n");
      // impratio: frictional-to-normal constraint impedance ratio. cone: pyramidal (default) or
      // elliptic friction cone. Both are key levers for foot slip under shear loads (see the
      // contact tuning notes below and MujocoSimulationParameters).
      mjcf.append("  <option gravity=\"0 0 -9.81\" integrator=\"implicitfast\" solver=\"Newton\" iterations=\"")
          .append(parameters.getSolverIterations())
          .append("\" noslip_iterations=\"").append(parameters.getNoslipIterations())
          .append("\" impratio=\"").append(parameters.getImpratio())
          .append("\" cone=\"").append(parameters.getUseEllipticFrictionCone() ? "elliptic" : "pyramidal")
          .append("\">\n");
      mjcf.append("    <flag filterparent=\"").append(parameters.getFilterParentCollisions() ? "enable" : "disable").append("\"/>\n");
      mjcf.append("  </option>\n");
      mjcf.append("  <compiler angle=\"radian\"/>\n");
      // Contact tuning (empirical, for closed-loop locomotion):
      // * friction "(slide torsional rolling)": slide is parameter-driven; rolling 0.01 (vs MuJoCo's
      //   0.0001) so spheres settle in finite time; torsional 0.05 balances turn-in-place authority
      //   against stance-foot yaw wobble.
      // * solref "<timeconst> <damp>": critically damped contact. 0.005 s is the empirical sweet
      //   spot -- 0.02 (MuJoCo default) is too soft for the controller; 0.002 gives harsh touchdowns.
      // * condim=4 (normal + 2D tangential + torsional): the flat foot makes 4 corner contacts, and
      //   per-corner torsional friction stops yaw-induced lateral foot slip (condim=3 wobbles).
      // Collision groups: the coarse foot box would self-collide (swing foot catching the stance
      // foot) in ways the real robot doesn't, so contype/conaffinity restrict robot geoms to test
      // only against terrain (robot=1/2, terrain=2/1), mirroring ContactPointBased.
      mjcf.append("  <default>\n");
      mjcf.append("    <geom friction=\"").append(parameters.getFrictionSlide()).append(" 0.05 0.01\" solref=\"")
          .append(parameters.getContactSolrefTimeconst()).append(" ").append(parameters.getContactSolrefDampRatio()).append("\" solimp=\"")
          .append(parameters.getContactSolimpDmin()).append(" ")
          .append(parameters.getContactSolimpDmax()).append(" 0.0007 0.5 2\" condim=\"4\"/>\n");
      mjcf.append("    <joint armature=\"").append(parameters.getJointArmature()).append("\"/>\n");
      mjcf.append("    <default class=\"robot\">\n");
      mjcf.append("      <geom contype=\"1\" conaffinity=\"2\"/>\n");
      mjcf.append("    </default>\n");
      mjcf.append("    <default class=\"terrain\">\n");
      mjcf.append("      <geom contype=\"2\" conaffinity=\"1\"/>\n");
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
         appendRobotBodies(mjcf, robotDefinition, ignoredJointNames, 2);
      }
      mjcf.append("  </worldbody>\n");
      if (parameters.getFilterParentCollisions())
      {
         for (Robot robot : robots)
         {
            appendParentChildContactExcludes(mjcf, robot.getRobotDefinition(), 2);
         }
      }
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
         if (ignoredJointNames.contains(jointDefinition.getName()))
            continue;
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

   private static void appendRobotBodies(StringBuilder sb, RobotDefinition robotDefinition, Set<String> ignoredJointNames, int indentLevel)
   {
      String namePrefix = robotDefinition.getName() + "_";
      List<JointDefinition> rootJoints = robotDefinition.getRootJointDefinitions();
      for (JointDefinition rootJoint : rootJoints)
      {
         appendBody(sb, rootJoint, rootJoint.getSuccessor(), namePrefix, ignoredJointNames, indentLevel);
      }
   }

   private static void appendBody(StringBuilder sb,
                                  JointDefinition joint,
                                  RigidBodyDefinition body,
                                  String namePrefix,
                                  Set<String> ignoredJointNames,
                                  int indent)
   {
      String pad = "  ".repeat(indent);

      sb.append(pad).append("<body name=\"").append(namePrefix).append(body.getName()).append('"');
      // For the root joint, place the body at its initial pose (MuJoCo uses the body's pos/quat
      // attributes as the starting qpos for the freejoint). Non-root joints use transformToParent
      // since the parent body's frame is the reference.
      RigidBodyTransform spawnTransform = computeSpawnTransform(joint);
      if (!MujocoTools.isIdentity(spawnTransform))
         sb.append(' ').append(MujocoTools.toPosQuatAttributes(spawnTransform));
      sb.append(">\n");

      MujocoTools.appendJoint(sb, joint, namePrefix, indent + 1);
      MujocoTools.appendInertial(sb, body, indent + 1);

      int geomIndex = 0;
      for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
      {
         MujocoTools.appendGeom(sb, "robot", namePrefix + body.getName() + "_geom_" + geomIndex, shape, indent + 1);
         geomIndex++;
      }

      for (JointDefinition childJoint : body.getChildrenJoints())
      {
         if (childJoint.getSuccessor() == null)
            continue;
         if (ignoredJointNames.contains(childJoint.getName()))
            continue;
         appendBody(sb, childJoint, childJoint.getSuccessor(), namePrefix, ignoredJointNames, indent + 1);
      }

      sb.append(pad).append("</body>\n");
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

   private static void appendParentChildContactExcludes(StringBuilder sb, RobotDefinition robotDefinition, int indent)
   {
      String namePrefix = robotDefinition.getName() + "_";
      String pad = "  ".repeat(indent);
      Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());
      StringBuilder excludes = new StringBuilder();
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         // Hands and other ignored subtrees are omitted from the MJCF (see appendBody); do not emit
         // <exclude> pairs that reference bodies MuJoCo never created.
         if (isJointOmittedFromMjcf(joint, ignoredJointNames))
            continue;

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
    * True when this joint (or an ancestor) is in {@code ignoredJointNames}, matching the subtree
    * pruning in {@link #appendBody}.
    */
   private static boolean isJointOmittedFromMjcf(JointDefinition joint, Set<String> ignoredJointNames)
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
