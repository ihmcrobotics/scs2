package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
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
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;

/**
 * Pure-Java MJCF generation helpers used by the MuJoCo SCS2 integration. No JNI in this class.
 */
public final class MujocoTools
{
   private MujocoTools()
   {
   }

   /**
    * MuJoCo XML uses {@code pos="x y z"} and {@code quat="w x y z"} (w-first). Converts a
    * {@link RigidBodyTransformReadOnly} into the MJCF attribute fragment
    * {@code "pos=\"x y z\" quat=\"w x y z\""}.
    */
   public static String toPosQuatAttributes(RigidBodyTransformReadOnly transform)
   {
      return toPosQuatAttributes(transform.getTranslation(), transform.getRotation());
   }

   /**
    * Same as {@link #toPosQuatAttributes(RigidBodyTransformReadOnly)} but takes the translation
    * and orientation separately. Accepts any {@link Orientation3DReadOnly} (quaternion, rotation
    * matrix, or axis-angle); a temporary {@link Quaternion} is used to pull out the (w, x, y, z)
    * components in the MuJoCo order.
    */
   public static String toPosQuatAttributes(Tuple3DReadOnly translation, Orientation3DReadOnly rotation)
   {
      Quaternion q = new Quaternion();
      q.set(rotation);
      return new StringBuilder()
         .append("pos=\"")
         .append(translation.getX()).append(' ')
         .append(translation.getY()).append(' ')
         .append(translation.getZ()).append("\" quat=\"")
         .append(q.getS()).append(' ')
         .append(q.getX()).append(' ')
         .append(q.getY()).append(' ')
         .append(q.getZ()).append('"')
         .toString();
   }

   /**
    * Emit a {@code <geom>} for one collision shape under the given collision class ("robot" or
    * "terrain"). Supports box / sphere / cylinder / capsule; meshes and convex polytopes would need
    * {@code <asset>} entries and emit a TODO comment instead.
    */
   static void appendGeom(StringBuilder sb, String geomClass, String name, CollisionShapeDefinition shape, int indent)
   {
      String pad = "  ".repeat(indent);
      GeometryDefinition geometry = shape.getGeometryDefinition();
      sb.append(pad).append("<geom class=\"").append(geomClass).append("\" name=\"").append(name).append('"');
      if (!isIdentity(shape.getOriginPose()))
         sb.append(' ').append(toPosQuatAttributes(shape.getOriginPose()));

      if (geometry instanceof Box3DDefinition box)
      {
         // MuJoCo box size is half-extents.
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
         // MuJoCo cylinder size: (radius, half-length).
         sb.append(" type=\"cylinder\" size=\"")
           .append(cylinder.getRadius()).append(' ')
           .append(cylinder.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         // MuJoCo capsule size: (radius, half-length-of-cylinder-section).
         sb.append(" type=\"capsule\" size=\"")
           .append(capsule.getRadiusX()).append(' ')
           .append(capsule.getLength() / 2.0).append("\"/>\n");
      }
      else
      {
         sb.append("/><!-- TODO unsupported geometry: ").append(geometry.getClass().getSimpleName()).append(" -->\n");
      }
   }

   /** Emit a body's {@code <inertial>} element (CoM offset, mass, and full inertia tensor). */
   static void appendInertial(StringBuilder sb, RigidBodyDefinition body, int indent)
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

   /** Emit a joint's MJCF element: {@code <freejoint>} for the root, {@code <joint>} for 1-DoF. */
   static void appendJoint(StringBuilder sb, JointDefinition joint, String namePrefix, int indent)
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

   /** True if the transform has neither rotation nor translation (so pos/quat can be omitted). */
   static boolean isIdentity(RigidBodyTransformReadOnly transform)
   {
      return !transform.hasRotation() && !transform.hasTranslation();
   }
}
