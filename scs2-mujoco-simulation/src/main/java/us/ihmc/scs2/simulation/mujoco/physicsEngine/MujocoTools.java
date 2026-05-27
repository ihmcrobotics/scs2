package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * Pure-Java helpers used by the MuJoCo SCS2 integration. No JNI in this class.
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
    * components in the MuJoCo order. Called only during MJCF assembly, not on the hot path.
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
}
