package us.ihmc.scs2.simulation.mujoco;

/**
 * Stable {@code mjtObj} values used by MuJoCo name/id lookup APIs.
 */
public final class MujocoObjectType
{
   public static final int BODY = 1;
   public static final int JOINT = 3;
   public static final int GEOM = 5;

   private MujocoObjectType()
   {
   }
}
