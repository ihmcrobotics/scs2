package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.scs2.simulation.mujoco.Mujoco;

/** MuJoCo {@code mjtSolver}, MJCF {@code option/@solver}. Default {@link #NEWTON}. */
public enum MujocoSolver
{
   PGS(Mujoco.mjSOL_PGS, "PGS"),
   CG(Mujoco.mjSOL_CG, "CG"),
   NEWTON(Mujoco.mjSOL_NEWTON, "Newton");

   private final int mujocoValue;
   private final String mjcfName;

   MujocoSolver(int mujocoValue, String mjcfName)
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

   public static MujocoSolver fromMujocoValue(int value)
   {
      for (MujocoSolver candidate : values())
      {
         if (candidate.mujocoValue == value)
            return candidate;
      }
      throw new IllegalArgumentException("Unknown mjtSolver value: " + value);
   }
}
