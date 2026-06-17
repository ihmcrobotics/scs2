package us.ihmc.scs2.simulation.mujoco;

import org.bytedeco.javacpp.BytePointer;

import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;

/**
 * Right-click -> Run in IntelliJ. Confirms:
 * - the MuJoCo native libraries load,
 * - a trivial MJCF compiles via mj_loadXML,
 * - 1000 mj_step calls drop the sphere under gravity.
 */
public class MujocoNativeSmokeMain
{
   private static final String TRIVIAL_MJCF = """
         <mujoco>
           <option timestep="0.002" gravity="0 0 -9.81"/>
           <worldbody>
             <geom type="plane" size="5 5 0.1" rgba="0.5 0.5 0.5 1"/>
             <body name="dropper" pos="0 0 1">
               <freejoint/>
               <geom type="sphere" size="0.1" mass="1"/>
             </body>
           </worldbody>
         </mujoco>
         """;

   public static void main(String[] args)
   {
      System.out.println("[smoke] Loading MuJoCo native libraries...");
      if (!MujocoNativeLibrary.load())
      {
         System.err.println("[smoke] FAILED to load MuJoCo native libraries.");
         System.exit(1);
      }
      System.out.println("[smoke] Native load OK.");

      // mj_loadXML wants a file path, not the XML content. Write the MJCF to a temp file.
      java.io.File mjcfFile;
      try
      {
         mjcfFile = java.io.File.createTempFile("mujoco-smoke-", ".xml");
         mjcfFile.deleteOnExit();
         java.nio.file.Files.writeString(mjcfFile.toPath(), TRIVIAL_MJCF);
      }
      catch (java.io.IOException e)
      {
         throw new RuntimeException(e);
      }

      BytePointer errorBuffer = new BytePointer(1000);
      mjModel model = Mujoco.mj_loadXML(mjcfFile.getAbsolutePath(), null, errorBuffer, 1000);
      if (model == null || model.isNull())
      {
         System.err.println("[smoke] mj_loadXML failed: " + errorBuffer.getString());
         System.exit(1);
      }
      System.out.println("[smoke] mj_loadXML OK. timestep=" + model.opt().timestep());

      mjData data = Mujoco.mj_makeData(model);
      if (data == null || data.isNull())
      {
         System.err.println("[smoke] mj_makeData failed.");
         System.exit(1);
      }

      double initialZ = data.qpos().get(2);
      System.out.println("[smoke] initial z=" + initialZ);

      for (int i = 0; i < 1000; i++)
         Mujoco.mj_step(model, data);

      double finalZ = data.qpos().get(2);
      System.out.println("[smoke] after 1000 steps: z=" + finalZ);

      if (!Double.isFinite(finalZ))
      {
         System.err.println("[smoke] FAILED: z is non-finite.");
         System.exit(1);
      }
      if (finalZ >= initialZ)
      {
         System.err.println("[smoke] FAILED: sphere did not drop. initial=" + initialZ + ", final=" + finalZ);
         System.exit(1);
      }

      Mujoco.mj_deleteData(data);
      Mujoco.mj_deleteModel(model);
      System.out.println("[smoke] PASS.");
   }
}
