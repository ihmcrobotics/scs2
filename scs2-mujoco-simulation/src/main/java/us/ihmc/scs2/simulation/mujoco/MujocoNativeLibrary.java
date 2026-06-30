package us.ihmc.scs2.simulation.mujoco;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class MujocoNativeLibrary implements NativeLibraryDescription
{
   @Override
   public String getPackage(OperatingSystem operatingSystem, Architecture architecture)
   {
      String architecturePackage = "";
      if (architecture == Architecture.x64)
      {
         architecturePackage = switch (operatingSystem)
         {
            case LINUX64 -> "linux-x86_64";
            case WIN64 -> "windows-x86_64";
            default -> "unknown";
         };
      }

      return "mujoco." + architecturePackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem operatingSystem, Architecture architecture)
   {
      switch (operatingSystem)
      {
         case LINUX64:
            // Filename must match the SONAME recorded in libjniMujoco.so by the linker, which is
            // the versioned MuJoCo library (verify with `readelf -d`). Bump in lockstep with the
            // MUJOCO_VERSION variable in install.sh / install.ps1.
            return NativeLibraryWithDependencies.fromFilename("libjniMujoco.so", "libmujoco.so.3.2.7");
         case WIN64:
            // On Windows the NativeLibraryLoader loads dependencies in declaration order before
            // the JNI bridge, so mujoco.dll is extracted and loaded first.
            return NativeLibraryWithDependencies.fromFilename("jniMujoco.dll", "mujoco.dll");
         default:
            break;
      }

      LogTools.warn("Unsupported platform: " + operatingSystem.name() + "-" + architecture.name());
      return null;
   }

   private static boolean loaded = false;

   public static boolean load()
   {
      if (!loaded)
      {
         MujocoNativeLibrary nativeLibrary = new MujocoNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(nativeLibrary);
      }
      return loaded;
   }
}
