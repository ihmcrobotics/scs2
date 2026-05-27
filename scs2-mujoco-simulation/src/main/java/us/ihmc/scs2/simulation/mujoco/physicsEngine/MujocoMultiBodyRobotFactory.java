package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.util.List;

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
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
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
      // MuJoCo's default rolling friction (0.0001) is far too low for an engine SCS2 users would
      // recognise -- spheres roll forever and don't settle. Bump rolling to 0.01 and torsional
      // to 0.1; sliding stays at the MuJoCo default 1.0.
      mjcf.append("  <default>\n");
      mjcf.append("    <geom friction=\"1.0 0.1 0.01\"/>\n");
      mjcf.append("  </default>\n");
      mjcf.append("  <worldbody>\n");
      for (TerrainObjectDefinition terrain : terrainObjects)
      {
         mjcf.append(MujocoTerrainFactory.toMjcfWorldbodyFragment(terrain));
      }
      for (Robot robot : robots)
      {
         appendRobotBodies(mjcf, robot.getRobotDefinition(), 2);
      }
      mjcf.append("  </worldbody>\n");
      mjcf.append("</mujoco>\n");
      return mjcf.toString();
   }

   /**
    * Register every joint in {@code robotDefinition} on the supplied {@link MujocoMultiBodyRobot}.
    * The world must already be compiled (model != null) before calling.
    */
   public static MujocoMultiBodyRobot registerJoints(RobotDefinition robotDefinition, mjModel model)
   {
      MujocoMultiBodyRobot mujocoRobot = new MujocoMultiBodyRobot(robotDefinition.getName(), model);

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         boolean isFloatingRoot = jointDefinition instanceof SixDoFJointDefinition
                                  && jointDefinition.getParentJoint() == null;
         try
         {
            mujocoRobot.registerJoint(jointDefinition.getName(), isFloatingRoot);
            System.out.println("[MujocoMultiBodyRobotFactory] registered joint '" + jointDefinition.getName()
                               + "' (isFloatingRoot=" + isFloatingRoot + ")");
         }
         catch (RuntimeException e)
         {
            System.err.println("[MujocoMultiBodyRobotFactory] SKIPPED joint '" + jointDefinition.getName() + "': " + e.getMessage());
         }
         // Also register the successor RigidBody so external wrench points can find their target.
         RigidBodyDefinition successor = jointDefinition.getSuccessor();
         if (successor != null)
            mujocoRobot.registerBody(successor.getName());
      }
      return mujocoRobot;
   }

   private static void appendRobotBodies(StringBuilder sb, RobotDefinition robotDefinition, int indentLevel)
   {
      List<JointDefinition> rootJoints = robotDefinition.getRootJointDefinitions();
      for (JointDefinition rootJoint : rootJoints)
      {
         appendBody(sb, rootJoint, rootJoint.getSuccessor(), indentLevel);
      }
   }

   private static void appendBody(StringBuilder sb, JointDefinition joint, RigidBodyDefinition body, int indent)
   {
      String pad = "  ".repeat(indent);

      sb.append(pad).append("<body name=\"").append(body.getName()).append('"');
      // For the root joint, place the body at its initial pose (MuJoCo uses the body's pos/quat
      // attributes as the starting qpos for the freejoint). Non-root joints use transformToParent
      // since the parent body's frame is the reference.
      RigidBodyTransform spawnTransform = computeSpawnTransform(joint);
      if (!isIdentity(spawnTransform))
         sb.append(' ').append(MujocoTools.toPosQuatAttributes(spawnTransform));
      sb.append(">\n");

      appendJoint(sb, joint, indent + 1);
      appendInertial(sb, body, indent + 1);

      int geomIndex = 0;
      for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
      {
         appendGeom(sb, body.getName() + "_geom_" + geomIndex, shape, indent + 1);
         geomIndex++;
      }

      // Recurse into child joints (this body is the predecessor for each child joint).
      for (JointDefinition childJoint : body.getChildrenJoints())
      {
         if (childJoint.getSuccessor() == null)
            continue;
         appendBody(sb, childJoint, childJoint.getSuccessor(), indent + 1);
      }

      sb.append(pad).append("</body>\n");
   }

   private static void appendJoint(StringBuilder sb, JointDefinition joint, int indent)
   {
      String pad = "  ".repeat(indent);
      if (joint instanceof SixDoFJointDefinition)
      {
         sb.append(pad).append("<freejoint name=\"").append(joint.getName()).append("\"/>\n");
      }
      else if (joint instanceof RevoluteJointDefinition rev)
      {
         appendOneDofJoint(sb, pad, "hinge", rev);
      }
      else if (joint instanceof PrismaticJointDefinition pris)
      {
         appendOneDofJoint(sb, pad, "slide", pris);
      }
      else
      {
         // Fixed, planar, spherical, cross-four-bar not supported in v1 -- the body just inherits
         // the parent body's frame statically.
         sb.append(pad).append("<!-- TODO unsupported joint type: ").append(joint.getClass().getSimpleName())
           .append(" (name=").append(joint.getName()).append(") -->\n");
      }
   }

   private static void appendOneDofJoint(StringBuilder sb, String pad, String mjcfType, OneDoFJointDefinition jointDef)
   {
      Tuple3DReadOnly axis = jointDef.getAxis();
      sb.append(pad).append("<joint name=\"").append(jointDef.getName())
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
      sb.append(pad).append("<geom name=\"").append(name).append('"');
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
