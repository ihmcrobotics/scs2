package us.ihmc.scs2.definition.robot.sdf;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Covers {@link SDFTools#tryToConvertToPath}'s last-resort fallback: every earlier strategy in that method assumes
 * the on-disk resource layout mirrors the {@code package://} URI's own path structure (e.g. a resource directory
 * containing a matching {@code robot_description/robots/.../assets/...} tree). The fallback drops that assumption -
 * it strips leading path segments off the URI one at a time and looks for the shrinking suffix directly under each
 * resource directory - so a mesh still resolves even when the on-disk bundle has been flattened relative to the
 * package path declared in the URDF (as happens with some downloaded log bundles).
 * <p>
 * Both tests use JUnit's {@code @TempDir} to fabricate a throwaway directory + a 1-byte placeholder file at test
 * time (not checked into source, deleted automatically afterward) - nothing here depends on any real robot's mesh
 * files or on-disk layout.
 * </p>
 */
public class SDFToolsTest
{
   /**
    * {@code tempDir} holds only {@code assets/merged/test.stl} - no {@code robot_description/robots/version/urdf/}
    * tree at all - while the requested URI is the full {@code package://robot_description/robots/version/urdf/
    * assets/merged/test.stl}. Every earlier resolution strategy in {@code tryToConvertToPath} would fail to find
    * this (the on-disk layout doesn't match the URI's path), so a successful result here specifically exercises the
    * suffix-stripping fallback - and its "prefer the longest/most-specific matching suffix" behavior, since
    * {@code assets/merged/test.stl} is the first (most specific) suffix that happens to exist under {@code tempDir}.
    */
   @Test
   public void testTryToConvertToPathWithFlattenedResourceLayout(@TempDir File tempDir) throws IOException
   {
      // Simulates an on-disk resource bundle that has been flattened relative to the ROS package structure
      // declared by the package:// URI, e.g. a downloaded log bundle with "assets/merged/*.stl" directly
      // alongside the URDF, instead of the full "robot_description/robots/version/urdf/assets/merged/" tree.
      File assetsDir = new File(tempDir, "assets/merged");
      assertEquals(true, assetsDir.mkdirs());
      File meshFile = new File(assetsDir, "test.stl");
      Files.write(meshFile.toPath(), new byte[] {0});

      String filename = "package://robot_description/robots/version/urdf/assets/merged/test.stl";
      List<String> resourceDirectories = new ArrayList<>(Collections.singletonList(tempDir.getAbsolutePath()));

      String result = SDFTools.tryToConvertToPath(filename, resourceDirectories, getClass().getClassLoader());

      assertEquals(meshFile.getAbsolutePath(), result);
   }

   /**
    * {@code tempDir} is empty this time - no file exists under any suffix of the requested URI, at any strip
    * length, down to just the bare filename. The fallback (and every strategy before it) must give up gracefully
    * and return {@code null} rather than throwing, so callers can report "mesh not found" instead of crashing.
    */
   @Test
   public void testTryToConvertToPathReturnsNullWhenUnresolvable(@TempDir File tempDir)
   {
      String filename = "package://robot_description/robots/version/urdf/assets/merged/does_not_exist.stl";
      List<String> resourceDirectories = new ArrayList<>(Collections.singletonList(tempDir.getAbsolutePath()));

      String result = SDFTools.tryToConvertToPath(filename, resourceDirectories, getClass().getClassLoader());

      assertNull(result);
   }
}
