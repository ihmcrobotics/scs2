package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
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
                                       double timestep)
   {
      if (!workingDirectory.exists() && !workingDirectory.mkdirs())
         throw new RuntimeException("Could not create MuJoCo working directory: " + workingDirectory);

      StringBuilder mjcf = new StringBuilder();
      mjcf.append("<mujoco>\n");
      mjcf.append("  <option timestep=\"").append(timestep)
          .append("\" gravity=\"0 0 -9.81\" solver=\"Newton\" iterations=\"100\"/>\n");
      // MuJoCo 3.x removed the `coordinate` attribute (local is the only mode). `angle="radian"`
      // is also the default in 3.x but kept here for clarity.
      mjcf.append("  <compiler angle=\"radian\"/>\n");
      // Contact tuning notes:
      // * friction: MuJoCo's default rolling friction (0.0001) is far too low for an engine SCS2
      //   users would recognise -- spheres roll forever. friction string is
      //   (sliding, torsional, rolling). Sliding stays at MuJoCo default 1.0. Torsional kept
      //   low (0.01) -- see the condim comment below. Rolling at 0.01 (vs default 0.0001) so
      //   the sphere demo settles in finite time; rolling is unused at condim=4 anyway.
      // * solref="0.005 1": critically damped contact with a 5 ms time constant. Default is
      //   (0.02, 1) which is too soft for closed-loop locomotion. Tried 0.002 to address visible
      //   CoP wandering from constraint-solver slack on the 4 foot corners; result was harder
      //   touchdown impacts that the IMU/state-estimator pipeline couldn't damp, walking time
      //   regressed from 138 s to ~18 s. 0.005 is the empirical sweet spot for both penetration
      //   (sub-mm) and impact smoothness.
      // * condim="4" + low torsional friction: normal + 2D tangential + torsional, no rolling.
      //   - Box-on-plane contact for a flat foot generates 4 contact points at the polygon
      //     corners (mjc_BoxBox face-overlap routine). condim=4 gives each corner its own
      //     torsional friction cone, summed across 4 corners during full stance.
      //   - condim=3 (no torsional) failed in testing: with only tangential friction, body yaw
      //     transferred into sideways foot slip via the 4 corners' coupled tangential constraints,
      //     producing a wobbly squat-walking gait and visible state-estimator drift through IMU.
      //   - condim=6 (adds rolling) prevented heel->sole pivoting during heel-strike: the heel
      //     point contact "catches" and the foot can't roll forward.
      //   - The torsional friction *value* matters: 0.1 made turn-in-place spiral outward
      //     (4 corners x large cone = body-yaw resistance bigger than controller authority).
      //     0.01 fixed turn-in-place but let the foot wobble freely during stance (visible as
      //     yaw-induced lateral motion of the 4 corner markers in the visualizer; controller
      //     fought it but eventually edge-loaded the foot and collapsed at ~132 s). 0.05 is
      //     the empirical compromise. Re-tune if either symptom resurfaces.
      // MuJoCo combines per-pair contact params: condim is max() of the two geoms, solref is
      // averaged. Applying to all geoms keeps contact behaviour consistent between robot-robot
      // and robot-terrain pairs.
      //
      // Collision groups: a humanoid's coarse collision hulls (especially the foot box, which
      // is a rectangular slab much larger than the actual mechanical foot edge) self-collide in
      // ways the real robot doesn't. Walking failure mode: swing-foot catches on the side of
      // the stance-foot box, controller can't push through, robot collapses. ContactPointBased
      // sidesteps this because its ground-contact points only test against the ground.
      // We replicate that here via contype/conaffinity bitmasks:
      //   robot   geoms: contype=1, conaffinity=2  -> tests only against group 2 (terrain)
      //   terrain geoms: contype=2, conaffinity=1  -> tests only against group 1 (robot)
      // A pair collides iff (contype1 & conaffinity2) != 0 OR (contype2 & conaffinity1) != 0.
      //   robot-robot:   (1&2)|(1&2) = 0  -> excluded
      //   robot-terrain: (1&1)|(2&2) = 3  -> tested
      //   terrain-terr.: (2&1)|(2&1) = 0  -> excluded (irrelevant; tiles don't overlap)
      // Selective self-collision (e.g. hand-on-torso for manipulation) is out of scope for v1.
      mjcf.append("  <default>\n");
      mjcf.append("    <geom friction=\"1.0 0.05 0.01\" solref=\"0.005 1\" condim=\"4\"/>\n");
      mjcf.append("    <default class=\"robot\">\n");
      mjcf.append("      <geom contype=\"1\" conaffinity=\"2\"/>\n");
      mjcf.append("    </default>\n");
      mjcf.append("    <default class=\"terrain\">\n");
      mjcf.append("      <geom contype=\"2\" conaffinity=\"1\"/>\n");
      mjcf.append("    </default>\n");
      mjcf.append("  </default>\n");
      mjcf.append("  <worldbody>\n");
      for (TerrainObjectDefinition terrain : terrainObjects)
      {
         mjcf.append(MujocoTerrainFactory.toMjcfWorldbodyFragment(terrain));
      }
      for (Robot robot : robots)
      {
         RobotDefinition robotDefinition = robot.getRobotDefinition();
         Set<String> ignoredJointNames = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());
         appendRobotBodies(mjcf, robotDefinition, ignoredJointNames, 2);
      }
      mjcf.append("  </worldbody>\n");
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
      if (!isIdentity(spawnTransform))
         sb.append(' ').append(MujocoTools.toPosQuatAttributes(spawnTransform));
      sb.append(">\n");

      appendJoint(sb, joint, namePrefix, indent + 1);
      appendInertial(sb, body, indent + 1);

      int geomIndex = 0;
      for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
      {
         appendGeom(sb, namePrefix + body.getName() + "_geom_" + geomIndex, shape, indent + 1);
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

   private static void appendJoint(StringBuilder sb, JointDefinition joint, String namePrefix, int indent)
   {
      String pad = "  ".repeat(indent);
      if (joint instanceof SixDoFJointDefinition)
      {
         sb.append(pad).append("<freejoint name=\"").append(namePrefix).append(joint.getName()).append("\"/>\n");
      }
      else if (joint instanceof RevoluteJointDefinition rev)
      {
         appendOneDofJoint(sb, pad, "hinge", namePrefix, rev);
      }
      else if (joint instanceof PrismaticJointDefinition pris)
      {
         appendOneDofJoint(sb, pad, "slide", namePrefix, pris);
      }
      else
      {
         sb.append(pad).append("<!-- TODO unsupported joint type: ").append(joint.getClass().getSimpleName())
           .append(" (name=").append(joint.getName()).append(") -->\n");
      }
   }

   private static void appendOneDofJoint(StringBuilder sb, String pad, String mjcfType, String namePrefix, OneDoFJointDefinition jointDef)
   {
      Tuple3DReadOnly axis = jointDef.getAxis();
      sb.append(pad).append("<joint name=\"").append(namePrefix).append(jointDef.getName())
        .append("\" type=\"").append(mjcfType)
        .append("\" axis=\"").append(axis.getX()).append(' ').append(axis.getY()).append(' ').append(axis.getZ())
        .append("\"/>\n");
   }

   private static void appendInertial(StringBuilder sb, RigidBodyDefinition body, int indent)
   {
      String pad = "  ".repeat(indent);
      Tuple3DReadOnly comOffset = body.getCenterOfMassOffset();
      var inertia = body.getMomentOfInertia();

      sb.append(pad).append("<inertial")
        .append(" pos=\"").append(comOffset.getX()).append(' ').append(comOffset.getY()).append(' ').append(comOffset.getZ()).append('"')
        .append(" mass=\"").append(body.getMass()).append('"');
      if (body.getMass() > 0.0)
      {
         // fullinertia order: Ixx Iyy Izz Ixy Ixz Iyz
         sb.append(" fullinertia=\"")
           .append(inertia.getM00()).append(' ')
           .append(inertia.getM11()).append(' ')
           .append(inertia.getM22()).append(' ')
           .append(inertia.getM01()).append(' ')
           .append(inertia.getM02()).append(' ')
           .append(inertia.getM12())
           .append('"');
      }
      sb.append("/>\n");
   }

   /**
    * Emit a {@code <geom>} for one collision shape. Used for both robot bodies (called here) and
    * terrain (called from {@link MujocoTerrainFactory}).
    */
   static void appendGeom(StringBuilder sb, String name, CollisionShapeDefinition shape, int indent)
   {
      String pad = "  ".repeat(indent);
      GeometryDefinition geometry = shape.getGeometryDefinition();
      sb.append(pad).append("<geom class=\"robot\" name=\"").append(name).append('"');
      if (!isIdentity(shape.getOriginPose()))
         sb.append(' ').append(MujocoTools.toPosQuatAttributes(shape.getOriginPose()));

      if (geometry instanceof Box3DDefinition box)
      {
         sb.append(" type=\"box\" size=\"")
           .append(box.getSizeX() / 2.0).append(' ')
           .append(box.getSizeY() / 2.0).append(' ')
           .append(box.getSizeZ() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Sphere3DDefinition sphere)
      {
         sb.append(" type=\"sphere\" size=\"").append(sphere.getRadius()).append("\"/>\n");
      }
      else if (geometry instanceof Cylinder3DDefinition cylinder)
      {
         sb.append(" type=\"cylinder\" size=\"")
           .append(cylinder.getRadius()).append(' ')
           .append(cylinder.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         sb.append(" type=\"capsule\" size=\"")
           .append(capsule.getRadiusX()).append(' ')
           .append(capsule.getLength() / 2.0).append("\"/>\n");
      }
      else
      {
         sb.append("/><!-- TODO unsupported geometry: ").append(geometry.getClass().getSimpleName()).append(" -->\n");
      }
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

   private static boolean isIdentity(RigidBodyTransform transform)
   {
      return !transform.hasRotation() && !transform.hasTranslation();
   }

   private static boolean isIdentity(us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly transform)
   {
      return !transform.hasRotation() && !transform.hasTranslation();
   }
}
