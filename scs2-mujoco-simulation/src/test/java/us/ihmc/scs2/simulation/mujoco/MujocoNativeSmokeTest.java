package us.ihmc.scs2.simulation.mujoco;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.bytedeco.javacpp.BytePointer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;

/**
 * Smoke test that exercises only the JavaCPP-generated binding (no SCS2 wiring). Confirms:
 * - the native library loads;
 * - a trivial MJCF compiles via mj_loadXML;
 * - 1000 steps complete and the world's qpos remains finite.
 *
 * <p>Cannot be run until {@code just install} + {@code just wrap} have produced
 * {@code Mujoco.java} and the resources {@code .so} files. See module README.
 */
public class MujocoNativeSmokeTest
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

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @Test
   public void canLoadAndStepATrivialMjcf() throws java.io.IOException
   {
      // mj_loadXML takes a *file path*, not XML content. Write the MJCF to a temp file first.
      java.nio.file.Path mjcfPath = java.nio.file.Files.createTempFile("mujoco-smoke-", ".xml");
      java.nio.file.Files.writeString(mjcfPath, TRIVIAL_MJCF);
      mjcfPath.toFile().deleteOnExit();

      BytePointer errorBuffer = new BytePointer(1000);
      mjModel model = Mujoco.mj_loadXML(mjcfPath.toAbsolutePath().toString(), null, errorBuffer, 1000);
      assertNotNull(model, "mj_loadXML returned null. Error: " + errorBuffer.getString());
      assertTrue(!model.isNull(), "mj_loadXML returned null model. Error: " + errorBuffer.getString());

      mjData data = Mujoco.mj_makeData(model);
      assertNotNull(data, "mj_makeData returned null");

      double initialZ = data.qpos().get(2);
      for (int i = 0; i < 1000; i++)
         Mujoco.mj_step(model, data);

      double finalZ = data.qpos().get(2);
      assertTrue(Double.isFinite(finalZ), "Sphere z position is non-finite after 1000 steps");
      assertTrue(finalZ < initialZ, "Sphere should have dropped under gravity (initial z=" + initialZ + ", final z=" + finalZ + ")");

      Mujoco.mj_deleteData(data);
      Mujoco.mj_deleteModel(model);
   }
}
