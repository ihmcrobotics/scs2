import org.apache.tools.ant.taskdefs.condition.Os
import us.ihmc.cd.LogTools

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-session-logger:source")
   api("us.ihmc:scs2-session-visualizer:source")

   var javaFXVersion = "17.0.9"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
   api(ihmc.javaFXModule("swing", javaFXVersion))

   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:ihmc-graphics-description:0.25.0")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:svgloader:0.0")
   api("us.ihmc:ihmc-javafx-extensions:17-0.2.1")
   api("us.ihmc:ihmc-messager-javafx:0.2.0")
   api("org.bytedeco:javacv-platform:1.5.9")

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
ihmc.jarWithLibFolder()
tasks.getByPath("installDist").dependsOn("compositeJar")
app.entrypoint(sessionVisualizerExecutableName, "us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer", listOf("-Djdk.gtk.version=2", "-Dprism.vsync=false"))

tasks.create("buildDebianPackage") {
   dependsOn("installDist")

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
         exclude(sessionVisualizerExecutableName)
      }.forEach(File::delete)

      val launchScriptFile = File("$sourceFolder/bin/$sessionVisualizerExecutableName")
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

      File("$baseFolder/DEBIAN").mkdirs()
      LogTools.info("Created directory $baseFolder/DEBIAN/: ${File("${baseFolder}/DEBIAN").exists()}")

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
         echo "-----------------------------------------------------------------------------------------"
         echo "---------------------------- Installation Notes: ----------------------------------------"
         echo "Add the following to your .bashrc to run SCS2 Session Visualizer form the command line:"
         echo "   export PATH=\${'$'}PATH:/opt/scs2-${ihmc.version}/bin/"
         echo "Then try to run the command '$sessionVisualizerExecutableName'"
         echo "-----------------------------------------------------------------------------------------"
         echo "-----------------------------------------------------------------------------------------"
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
         exec {
            commandLine("chmod", "+x", "$baseFolder/DEBIAN/postinst")
         }
         exec {
            commandLine("chmod", "+x", "$sourceFolder/bin/$sessionVisualizerExecutableName")
         }
         exec {
            workingDir(File(debianFolder))
            commandLine("dpkg", "--build", "scs2-${ihmc.version}")
         }
      }
   }
}