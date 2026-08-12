package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

import us.ihmc.scs2.simulation.mujoco.Mujoco;

/**
 * MuJoCo {@code mjtCone}, MJCF {@code option/@cone}. Elliptic + large {@code impratio} + Newton is
 * MuJoCo's documented remedy for contact slip.
 */
public enum MujocoCone
{
   PYRAMIDAL(Mujoco.mjCONE_PYRAMIDAL, "pyramidal"),
   ELLIPTIC(Mujoco.mjCONE_ELLIPTIC, "elliptic");

   private final int mujocoValue;
   private final String mjcfName;

   MujocoCone(int mujocoValue, String mjcfName)
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

   public static MujocoCone fromMujocoValue(int value)
   {
      for (MujocoCone candidate : values())
      {
         if (candidate.mujocoValue == value)
            return candidate;
      }
      throw new IllegalArgumentException("Unknown mjtCone value: " + value);
   }
}
