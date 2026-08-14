package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.scs2.simulation.mujoco.Mujoco;

/**
 * MuJoCo {@code mjtIntegrator}, MJCF {@code option/@integrator}. MuJoCo defaults to {@link #EULER};
 * SCS2 seeds {@link #IMPLICITFAST}, more stable for joint-damping-heavy humanoids.
 */
public enum MujocoIntegrator
{
   EULER(Mujoco.mjINT_EULER, "Euler"),
   RK4(Mujoco.mjINT_RK4, "RK4"),
   IMPLICIT(Mujoco.mjINT_IMPLICIT, "implicit"),
   IMPLICITFAST(Mujoco.mjINT_IMPLICITFAST, "implicitfast");

   private final int mujocoValue;
   private final String mjcfName;

   MujocoIntegrator(int mujocoValue, String mjcfName)
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

   public static MujocoIntegrator fromMujocoValue(int value)
   {
      for (MujocoIntegrator candidate : values())
      {
         if (candidate.mujocoValue == value)
            return candidate;
      }
      throw new IllegalArgumentException("Unknown mjtIntegrator value: " + value);
   }
}
