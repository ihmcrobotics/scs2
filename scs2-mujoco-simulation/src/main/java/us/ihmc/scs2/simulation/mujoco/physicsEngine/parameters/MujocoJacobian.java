package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.scs2.simulation.mujoco.Mujoco;

/** MuJoCo {@code mjtJacobian}, MJCF {@code option/@jacobian}. Performance only. */
public enum MujocoJacobian
{
   DENSE(Mujoco.mjJAC_DENSE, "dense"),
   SPARSE(Mujoco.mjJAC_SPARSE, "sparse"),
   AUTO(Mujoco.mjJAC_AUTO, "auto");

   private final int mujocoValue;
   private final String mjcfName;

   MujocoJacobian(int mujocoValue, String mjcfName)
   {
      this.mujocoValue = mujocoValue;
      this.mjcfName = mjcfName;
   }

   public int toMujocoValue()
   {
      return mujocoValue;
   }

   public String getMjcfName()
   {
      return mjcfName;
   }

   public static MujocoJacobian fromMujocoValue(int value)
   {
      for (MujocoJacobian candidate : values())
      {
         if (candidate.mujocoValue == value)
            return candidate;
      }
      throw new IllegalArgumentException("Unknown mjtJacobian value: " + value);
   }
}
