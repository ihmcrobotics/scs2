import org.apache.tools.ant.taskdefs.condition.Os

plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

allprojects {
   tasks.javadoc {
      exclude("us/ihmc/**")
   }
}

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-session-logger:source")
   api("us.ihmc:scs2-session-visualizer:source")

   var javaFXVersion = "17.0.8"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
   api(ihmc.javaFXModule("swing", javaFXVersion))

   api("us.ihmc:euclid:0.22.5")
   api("us.ihmc:euclid-shape:0.22.5")
   api("us.ihmc:euclid-frame:0.22.5")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:ihmc-javafx-extensions:17-0.2.2")
   api("us.ihmc:ihmc-messager-javafx:0.2.1")

   api("org.reflections:reflections:0.9.11")

   // JavaFX extensions
   api("org.controlsfx:controlsfx:11.1.0")
   // TODO Switch away from the de.jensd to ikonli
   api("de.jensd:fontawesomefx-commons:9.1.2")
   api("de.jensd:fontawesomefx-octicons:4.3.0-9.1.2")
   api("de.jensd:fontawesomefx-materialicons:2.2.0-9.1.2")
   api("de.jensd:fontawesomefx-materialdesignfont:2.0.26-9.1.2")
   api("org.kordamp.ikonli:ikonli-javafx:12.3.1")
   api("org.kordamp.ikonli:ikonli-fontawesome-pack:12.3.1")
   api("org.kordamp.ikonli:ikonli-fontawesome5-pack:12.3.1")
   api("us.ihmc:jfoenix:17-0.1.1")
   api("org.apache.commons:commons-text:1.9")

   api("us.ihmc:jim3dsModelImporterJFX:0.7")
   api("us.ihmc:jimColModelImporterJFX:0.6")
   api("us.ihmc:jimFxmlModelImporterJFX:0.5")
   api("us.ihmc:jimObjModelImporterJFX:0.8")
   api("us.ihmc:jimStlMeshImporterJFX:0.7")
   api("us.ihmc:jimX3dModelImporterJFX:0.4")

   // Dependencies for checking the version
   api("com.squareup.okhttp3:okhttp:4.12.0")
   api("com.google.code.gson:gson:2.10.1")

   api("me.tongfei:progressbar:0.10.0")
   api("commons-cli:commons-cli:1.6.0")
}

testDependencies {
   api("org.apache.commons:commons-math:2.2")
   api("org.testfx:openjfx-monocle:17.0.10")
   api("org.testfx:testfx-core:4.0.18")

}

categories.configure("javafx-headless")
{
   jvmArguments += "-Dtestfx.headless=true"
   jvmArguments += "-Dtestfx.robot=glass"
   jvmArguments += "-Djava.awt.headless=true"
   jvmArguments += "-Dprism.order=sw"
   jvmArguments += "-Dprism.verbose=true"
}

val sessionVisualizerExecutableName = "SCS2SessionVisualizer"
val mcapRepackAppExecutableName = "MCAPRepackApplication"
ihmc.jarWithLibFolder()
tasks.getByPath("installDist").dependsOn("compositeJar")
app.entrypoint(sessionVisualizerExecutableName, "us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer", listOf("-Djdk.gtk.version=2", "-Dprism.vsync=false"))
app.entrypoint(mcapRepackAppExecutableName, "us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.MCAPRepackApplication", listOf("-Djdk.gtk.version=2", "-Dprism.vsync=false"))

/**
 * This task is used to compile the project and filter out any dependency not required for Linux.
 */
tasks.register("installDistLinux") {
   dependsOn("installDist")

   doLast() {
      fileTree("${project.projectDir}/build/install/scs2-session-visualizer-jfx/lib").matching {
         include("*-win.jar")
         include("*-android-*")
         include("*-windows-*")
         include("*-ios-*")
         include("*-macosx-*")
         include("*-osx-*")
      }.forEach(File::delete)
   }
}

tasks.register("buildDebianPackage") {
   dependsOn("installDistLinux")

   doLast {
      val deploymentFolder = "${project.projectDir}/deployment"

      val debianFolder = "$deploymentFolder/debian"
      File(debianFolder).deleteRecursively()

      val baseFolder = "$deploymentFolder/debian/scs2-${ihmc.version}"
      val sourceFolder = "$baseFolder/opt/scs2-${ihmc.version}/"

      copy {
         from("${project.projectDir}/src/main/resources/icons/scs-icon.png")
         into("$sourceFolder/icon/")
      }

      copy {
         from("${project.projectDir}/build/install/scs2-session-visualizer-jfx/")
         into(sourceFolder)
      }

      fileTree("$sourceFolder/bin").matching {
         exclude(sessionVisualizerExecutableName, mcapRepackAppExecutableName)
      }.forEach(File::delete)

      addVSyncLinuxHackForJavaFXApp(sourceFolder, sessionVisualizerExecutableName)
      addVSyncLinuxHackForJavaFXApp(sourceFolder, mcapRepackAppExecutableName)

      File("$baseFolder/DEBIAN").mkdirs()
      println("Created directory $baseFolder/DEBIAN/: ${File("${baseFolder}/DEBIAN").exists()}")

      File("$baseFolder/DEBIAN/control").writeText(
            """
         Package: scs2
         Version: ${ihmc.version}
         Section: base
         Architecture: all
         Depends: default-jre (>= 2:1.17) | java17-runtime
         Maintainer: Sylvain Bertrand <sbertrand@ihmc.org>
         Description: Session Visualizer for SCS2
         Homepage: ${ihmc.vcsUrl}
         
         """.trimIndent()
      )

      File("$baseFolder/DEBIAN/postinst").writeText(
            """
         #!/bin/bash
         # Without this, the desktop file does not appear in the system menu.
         sudo desktop-file-install /usr/share/applications/scs2-${ihmc.version}-visualizer.desktop
         echo "-----------------------------------------------------------------------------------------------------------------------"
         echo "----------------------------------------------- Installation Notes: ---------------------------------------------------"
         echo "Add the following to your .bashrc to run SCS2 Session Visualizer form the command line:"
         echo "   export PATH=\${'$'}PATH:/opt/scs2-${ihmc.version}/bin/"
         echo "Then run the command '$sessionVisualizerExecutableName' to start the SCS2 Session Visualizer."
         echo "You can also run '$mcapRepackAppExecutableName' to start the MCAP Repack Application to help with corrupted MCAP files."
         echo "-----------------------------------------------------------------------------------------------------------------------"
         echo "-----------------------------------------------------------------------------------------------------------------------"
         """.trimIndent()
      )

      File("$baseFolder/usr/share/applications/").mkdirs()
      File("$baseFolder/usr/share/applications/scs2-${ihmc.version}-visualizer.desktop").writeText(
            """
         [Desktop Entry]
         Name=SCS2 Session Visualizer
         Comment=Session Visualizer for SCS2
         Exec=/opt/scs2-${ihmc.version}/bin/$sessionVisualizerExecutableName
         Icon=/opt/scs2-${ihmc.version}/icon/scs-icon.png
         Version=1.0
         Terminal=true
         Type=Application
         Categories=Utility;Application;
         """.trimIndent()
      )

      if (Os.isFamily(Os.FAMILY_UNIX))
      {
         ihmc.exec(ProcessBuilder("chmod", "+x", "$baseFolder/DEBIAN/postinst"))
         ihmc.exec(ProcessBuilder("chmod", "+x", "$sourceFolder/bin/$sessionVisualizerExecutableName"))
         ihmc.exec(ProcessBuilder("chmod", "+x", "$sourceFolder/bin/$mcapRepackAppExecutableName"))
         ihmc.exec(ProcessBuilder("dpkg", "--build", "scs2-${ihmc.version}").directory(File(debianFolder)))
      }
   }
}

fun addVSyncLinuxHackForJavaFXApp(sourceFolder: String, javafxappname: String)
{
   val launchScriptFile = File("$sourceFolder/bin/$javafxappname")
   var originalScript = launchScriptFile.readText()
   originalScript = originalScript.replaceFirst(
         "#!/bin/sh", """
         #!/bin/bash
         # This is a workaround for a bug in JavaFX 17.0.1, disabling vsync to improve framerate with multiple windows.
         export __GL_SYNC_TO_VBLANK=0

      """.trimIndent()
   )

   launchScriptFile.delete()
   launchScriptFile.writeText(originalScript)
}

// Stable upgrade UUID for the Windows MSI. Must never change across releases:
// rotating it would orphan existing installations on user machines.
val windowsUpgradeUuid = "f74c546b-1d1d-4114-9812-809b8eb1412c"
val windowsDeploymentRoot = "${project.projectDir}/deployment/windows"
val windowsStagingDir = "$windowsDeploymentRoot/staging"
val windowsLaunchersDir = "$windowsDeploymentRoot/launchers"
val windowsAppImageDir = "$windowsDeploymentRoot/app-image"
val windowsMsiDir = "$windowsDeploymentRoot/msi"
val windowsAppImageJlinkDir = "$windowsDeploymentRoot/app-image-jlink"
val windowsMsiJlinkDir = "$windowsDeploymentRoot/msi-jlink"
val jlinkRuntimeDir = "${project.projectDir}/build/jlink-runtime"
val windowsIcon = "${project.projectDir}/src/main/resources/icons/scs-icon.ico"
val windowsMainClass = "us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer"
val mcapRepackMainClass = "us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.MCAPRepackApplication"

// Module list from docs/executable-plan.md §2.4. Discovered via `jdeps
// --print-module-deps` and expanded with modules touched indirectly
// (HTTPS, Swing interop, sun.misc.Unsafe, service-loader providers).
// jdk.zipfs registers the "jar"/"zip" FileSystemProvider used by
// YoGraphicFXResourceManager via FileSystems.newFileSystem(jar:...).
//
// JavaFX is intentionally NOT in this list. The IHMC distribution
// patches `javafx.scene.chart` from a classpath jar
// (ihmc-javafx-extensions adds `FastAxisBase`); baking the JavaFX
// jmods into the boot module layer would give `javafx.controls`
// ownership of that package and the JVM would reject the classpath
// patch with NoClassDefFoundError. The full JavaFX 17.0.8 runtime
// (including native DLLs) is shipped as classpath jars
// (`javafx-*-17.0.8-win.jar`) alongside the application, matching
// the Stage 1 layout.
val jlinkAddModules = listOf(
      "java.base", "java.compiler", "java.datatransfer", "java.desktop",
      "java.logging", "java.management", "java.naming", "java.net.http",
      "java.prefs", "java.rmi", "java.scripting", "java.security.jgss",
      "java.sql", "java.xml",
      "jdk.crypto.cryptoki", "jdk.crypto.ec", "jdk.localedata",
      "jdk.unsupported", "jdk.unsupported.desktop", "jdk.zipfs"
).joinToString(",")

fun requireWindowsHost()
{
   if (!Os.isFamily(Os.FAMILY_WINDOWS))
      throw GradleException("Windows packaging tasks only run on Windows.")
}

fun requireJdk17Plus()
{
   if (JavaVersion.current() < JavaVersion.VERSION_17)
      throw GradleException("Windows packaging tasks require JDK 17 or newer (running on ${JavaVersion.current()}).")
}

// MSI ProductVersion only honours the first three numeric components for
// upgrade detection (the fourth is silently ignored). The IHMC Gradle
// version is `<java-baseline>-<semver>` (e.g. `17-0.32.1`); we strip the
// java-baseline prefix so the three significant fields land on the actual
// semver (0.32.1) and patch-level releases differentiate correctly.
fun windowsInstallerVersion() = ihmc.version.substringAfter("-")
fun windowsMainJar() = "${project.name}-${ihmc.version}.jar"

fun writeMcapLauncherProperties()
{
   File(windowsLaunchersDir).mkdirs()
   // Properties-file format treats `\` as an escape character; use forward
   // slashes (accepted by jpackage on Windows) for the icon path.
   File("$windowsLaunchersDir/$mcapRepackAppExecutableName.properties").writeText(
         """
         main-jar=${windowsMainJar()}
         main-class=$mcapRepackMainClass
         win-console=true
         win-menu=false
         win-shortcut=false
         icon=${windowsIcon.replace("\\", "/")}
         java-options=-Dprism.vsync=false
         java-options=-Xmx2g
         """.trimIndent()
   )
}

// Resolve a JDK tool from the running JDK's bin/ when available, otherwise
// fall back to PATH. Gives a clear "missing tool" error on JREs/JDKs that do
// not ship the tool (e.g. JBR), instead of an obscure ProcessBuilder failure.
fun jdkToolExecutable(toolName: String): String
{
   val javaHome = System.getProperty("java.home") ?: return toolName
   val candidates = listOf(File(javaHome, "bin/$toolName.exe"), File(javaHome, "bin/$toolName"))
   return candidates.firstOrNull { it.isFile }?.absolutePath ?: toolName
}

fun jpackageExecutable() = jdkToolExecutable("jpackage")
fun jlinkExecutable()    = jdkToolExecutable("jlink")

fun jpackageArgsCommon(type: String, dest: String, runtimeImage: String? = null): List<String>
{
   val args = mutableListOf(
         jpackageExecutable(),
         "--type", type,
         "--name", sessionVisualizerExecutableName,
         "--app-version", windowsInstallerVersion(),
         "--vendor", "IHMC",
         "--description", "Simulation Construction Set 2 - Session Visualizer",
         "--copyright", "IHMC",
         "--input", "$windowsStagingDir/lib",
         "--dest", dest,
         "--main-jar", windowsMainJar(),
         "--main-class", windowsMainClass,
         "--icon", windowsIcon,
         "--java-options", "-Dprism.vsync=false",
         "--java-options", "-Xmx8g"
   )
   if (runtimeImage != null)
      args += listOf("--runtime-image", runtimeImage)
   args += listOf(
         "--add-launcher",
         "$mcapRepackAppExecutableName=$windowsLaunchersDir/$mcapRepackAppExecutableName.properties"
   )
   return args
}

/**
 * Stages the installDist output for Windows packaging by copying it to a clean
 * staging directory and removing native classifier jars for other platforms.
 */
tasks.register("installDistWindows") {
   dependsOn("installDist")

   doFirst {
      requireWindowsHost()
   }

   doLast {
      File(windowsStagingDir).deleteRecursively()
      copy {
         from("${project.projectDir}/build/install/scs2-session-visualizer-jfx/")
         into(windowsStagingDir)
      }
      fileTree("$windowsStagingDir/lib").matching {
         include("*-linux-*")
         include("*-linux.jar")
         include("*-android-*")
         include("*-ios-*")
         include("*-macos-*")
         include("*-osx-*")
         // Opt-in via -PexcludeOpenCvGpu=true. Drops the 132 MB CUDA-enabled
         // OpenCV native classifier jar. The companion CPU classifier
         // (`*-windows-x86_64.jar`) is retained. The only SCS2 code path that
         // touches OpenCV is ZEDSVOVideoDataReader, which uses the CPU
         // opencv_core API only. See docs/executable-plan.md §2.8.
         if (findProperty("excludeOpenCvGpu")?.toString() == "true")
            include("*-windows-x86_64-gpu*")
      }.forEach(File::delete)
   }
}

tasks.register("packageWindowsAppImage") {
   dependsOn("installDistWindows")

   doFirst {
      requireWindowsHost()
      requireJdk17Plus()
   }

   doLast {
      File(windowsAppImageDir).deleteRecursively()
      File(windowsAppImageDir).mkdirs()
      writeMcapLauncherProperties()
      ihmc.exec(ProcessBuilder(jpackageArgsCommon("app-image", windowsAppImageDir)))
   }
}

tasks.register("packageWindowsMsi") {
   dependsOn("installDistWindows")

   doFirst {
      requireWindowsHost()
      requireJdk17Plus()
   }

   doLast {
      File(windowsMsiDir).deleteRecursively()
      File(windowsMsiDir).mkdirs()
      writeMcapLauncherProperties()
      val args = jpackageArgsCommon("msi", windowsMsiDir) + listOf(
            "--win-dir-chooser",
            "--win-menu",
            "--win-menu-group", "IHMC",
            "--win-shortcut",
            "--win-upgrade-uuid", windowsUpgradeUuid
      )
      ihmc.exec(ProcessBuilder(args))
   }
}

tasks.register("buildWindowsPackages") {
   dependsOn("packageWindowsAppImage", "packageWindowsMsi")
}

/**
 * Builds a trimmed JRE image using jlink containing only the JDK modules
 * required by the application. JavaFX is intentionally shipped as classpath
 * jars (see jlinkAddModules comment). See docs/executable-plan.md §2.4.
 */
tasks.register("buildJlinkRuntime") {
   doFirst {
      requireWindowsHost()
      requireJdk17Plus()
   }

   doLast {
      val javaHome = System.getProperty("java.home")
            ?: throw GradleException("java.home system property is not set.")
      val modulePath = "$javaHome/jmods"

      File(jlinkRuntimeDir).deleteRecursively()

      ihmc.exec(ProcessBuilder(
            jlinkExecutable(),
            "--module-path", modulePath,
            "--add-modules", jlinkAddModules,
            "--strip-debug",
            "--no-man-pages",
            "--no-header-files",
            "--compress=2",
            "--include-locales=en",
            "--output", jlinkRuntimeDir
      ))
   }
}

tasks.register("packageWindowsAppImageJlink") {
   dependsOn("installDistWindows", "buildJlinkRuntime")

   doFirst {
      requireWindowsHost()
      requireJdk17Plus()
   }

   doLast {
      File(windowsAppImageJlinkDir).deleteRecursively()
      File(windowsAppImageJlinkDir).mkdirs()
      writeMcapLauncherProperties()
      ihmc.exec(ProcessBuilder(
            jpackageArgsCommon("app-image", windowsAppImageJlinkDir, jlinkRuntimeDir)))
   }
}

tasks.register("packageWindowsMsiJlink") {
   dependsOn("installDistWindows", "buildJlinkRuntime")

   doFirst {
      requireWindowsHost()
      requireJdk17Plus()
   }

   doLast {
      File(windowsMsiJlinkDir).deleteRecursively()
      File(windowsMsiJlinkDir).mkdirs()
      writeMcapLauncherProperties()
      val args = jpackageArgsCommon("msi", windowsMsiJlinkDir, jlinkRuntimeDir) + listOf(
            "--win-dir-chooser",
            "--win-menu",
            "--win-menu-group", "IHMC",
            "--win-shortcut",
            "--win-upgrade-uuid", windowsUpgradeUuid
      )
      ihmc.exec(ProcessBuilder(args))
   }
}

tasks.register("buildWindowsPackagesJlink") {
   dependsOn("packageWindowsAppImageJlink", "packageWindowsMsiJlink")
}

// Stable bundle identifier for the macOS app. Must never change across releases: it's the macOS/PKG
// analog of windowsUpgradeUuid above, and rotating it would make macOS (and users' Dock/Spotlight
// entries) treat a new version as a different, unrelated app.
val macPackageIdentifier = "us.ihmc.scs2.sessionvisualizer"
val macDeploymentRoot = "${project.projectDir}/deployment/mac"
val macStagingDir = "$macDeploymentRoot/staging"
val macLaunchersDir = "$macDeploymentRoot/launchers"
val macAppImageDir = "$macDeploymentRoot/app-image"
val macDmgDir = "$macDeploymentRoot/dmg"
val macAppImageJlinkDir = "$macDeploymentRoot/app-image-jlink"
val macDmgJlinkDir = "$macDeploymentRoot/dmg-jlink"
val macIcon = "${project.projectDir}/src/main/resources/icons/scs-icon.icns"

// jpackage on macOS rejects an --app-version whose first component is zero (Info.plist versioning
// requires a positive leading integer); windowsInstallerVersion()'s "0.33.1" is invalid here since this
// project's semver major is currently always 0. Substitute the java-baseline prefix (stable across
// releases) as the leading component instead, e.g. "17-0.33.1" -> "17.33.1".
fun macInstallerVersion(): String
{
   val javaBaseline = ihmc.version.substringBefore("-")
   val semverTail = ihmc.version.substringAfter("-").split(".").drop(1).joinToString(".")
   return "$javaBaseline.$semverTail"
}

fun requireMacHost()
{
   if (!Os.isFamily(Os.FAMILY_MAC))
      throw GradleException("macOS packaging tasks only run on macOS.")
}

fun writeMcapLauncherPropertiesMac()
{
   File(macLaunchersDir).mkdirs()
   File("$macLaunchersDir/$mcapRepackAppExecutableName.properties").writeText(
         """
         main-jar=${windowsMainJar()}
         main-class=$mcapRepackMainClass
         icon=$macIcon
         java-options=-Dprism.vsync=false
         java-options=-Xmx2g
         """.trimIndent()
   )
}

fun jpackageArgsCommonMac(type: String, dest: String, runtimeImage: String? = null): List<String>
{
   val args = mutableListOf(
         jpackageExecutable(),
         "--type", type,
         "--name", sessionVisualizerExecutableName,
         "--app-version", macInstallerVersion(),
         "--vendor", "IHMC",
         "--description", "Simulation Construction Set 2 - Session Visualizer",
         "--copyright", "IHMC",
         "--input", "$macStagingDir/lib",
         "--dest", dest,
         "--main-jar", windowsMainJar(),
         "--main-class", windowsMainClass,
         "--icon", macIcon,
         "--mac-package-identifier", macPackageIdentifier,
         "--mac-package-name", "SCS2 Session Visualizer",
         "--java-options", "-Dprism.vsync=false",
         "--java-options", "-Xmx8g"
   )
   if (runtimeImage != null)
      args += listOf("--runtime-image", runtimeImage)
   args += listOf(
         "--add-launcher",
         "$mcapRepackAppExecutableName=$macLaunchersDir/$mcapRepackAppExecutableName.properties"
   )
   return args
}

/**
 * Stages the installDist output for macOS packaging by copying it to a clean staging directory and
 * removing native classifier jars for other platforms. Unlike Linux/Windows, there is no OpenCV/ZED
 * native classifier to keep or drop here: the ZED SDK has no macOS build at all, so no mac-classified
 * artifact for it is ever resolved in the first place (see MultiVideoDataReader's guard around
 * ZEDSVOScrubber.findZEDSensorDatFiles for the corresponding runtime fallback).
 */
tasks.register("installDistMac") {
   dependsOn("installDist")

   doFirst {
      requireMacHost()
   }

   doLast {
      File(macStagingDir).deleteRecursively()
      copy {
         from("${project.projectDir}/build/install/scs2-session-visualizer-jfx/")
         into(macStagingDir)
      }
      fileTree("$macStagingDir/lib").matching {
         include("*-win.jar")
         include("*-windows-*")
         include("*-linux-*")
         include("*-linux.jar")
         include("*-android-*")
         include("*-ios-*")
      }.forEach(File::delete)
   }
}

tasks.register("packageMacAppImage") {
   dependsOn("installDistMac")

   doFirst {
      requireMacHost()
      requireJdk17Plus()
   }

   doLast {
      File(macAppImageDir).deleteRecursively()
      File(macAppImageDir).mkdirs()
      writeMcapLauncherPropertiesMac()
      ihmc.exec(ProcessBuilder(jpackageArgsCommonMac("app-image", macAppImageDir)))
   }
}

tasks.register("packageMacDmg") {
   dependsOn("installDistMac")

   doFirst {
      requireMacHost()
      requireJdk17Plus()
   }

   doLast {
      File(macDmgDir).deleteRecursively()
      File(macDmgDir).mkdirs()
      writeMcapLauncherPropertiesMac()
      ihmc.exec(ProcessBuilder(jpackageArgsCommonMac("dmg", macDmgDir)))
   }
}

tasks.register("buildMacPackages") {
   dependsOn("packageMacAppImage", "packageMacDmg")
}

/**
 * Builds a trimmed JRE image for macOS using jlink, same module list and same JavaFX-stays-on-classpath
 * reasoning as buildJlinkRuntime above.
 */
tasks.register("buildJlinkRuntimeMac") {
   doFirst {
      requireMacHost()
      requireJdk17Plus()
   }

   doLast {
      val javaHome = System.getProperty("java.home")
            ?: throw GradleException("java.home system property is not set.")
      val modulePath = "$javaHome/jmods"

      File(jlinkRuntimeDir).deleteRecursively()

      ihmc.exec(ProcessBuilder(
            jlinkExecutable(),
            "--module-path", modulePath,
            "--add-modules", jlinkAddModules,
            "--strip-debug",
            "--no-man-pages",
            "--no-header-files",
            "--compress=2",
            "--include-locales=en",
            "--output", jlinkRuntimeDir
      ))
   }
}

tasks.register("packageMacAppImageJlink") {
   dependsOn("installDistMac", "buildJlinkRuntimeMac")

   doFirst {
      requireMacHost()
      requireJdk17Plus()
   }

   doLast {
      File(macAppImageJlinkDir).deleteRecursively()
      File(macAppImageJlinkDir).mkdirs()
      writeMcapLauncherPropertiesMac()
      ihmc.exec(ProcessBuilder(
            jpackageArgsCommonMac("app-image", macAppImageJlinkDir, jlinkRuntimeDir)))
   }
}

tasks.register("packageMacDmgJlink") {
   dependsOn("installDistMac", "buildJlinkRuntimeMac")

   doFirst {
      requireMacHost()
      requireJdk17Plus()
   }

   doLast {
      File(macDmgJlinkDir).deleteRecursively()
      File(macDmgJlinkDir).mkdirs()
      writeMcapLauncherPropertiesMac()
      ihmc.exec(ProcessBuilder(
            jpackageArgsCommonMac("dmg", macDmgJlinkDir, jlinkRuntimeDir)))
   }
}

tasks.register("buildMacPackagesJlink") {
   dependsOn("packageMacAppImageJlink", "packageMacDmgJlink")
}
