plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.23"
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

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
   api(ihmc.javaFXModule("swing", javaFXVersion))

   api("us.ihmc:ihmc-javafx-toolkit:17-0.21.4") {
      exclude(group="us.ihmc", module="jassimp")
      exclude(group="us.ihmc", module="euclid")
      exclude(group="us.ihmc", module="euclid-shape")
      exclude(group="us.ihmc", module="euclid-frame")
   }
   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:euclid-shape:0.19.1")
   api("us.ihmc:euclid-frame:0.19.1")
   api("us.ihmc:ihmc-graphics-description:0.20.0")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:svgloader:0.0")
   api("us.ihmc:ihmc-javafx-extensions:17-0.1.3")
   api("us.ihmc:ihmc-messager-javafx:0.2.0")

   api("org.reflections:reflections:0.9.11")

   // JavaFX extensions
   api("org.controlsfx:controlsfx:11.1.0")
   api("de.jensd:fontawesomefx-commons:9.1.2")
   api("de.jensd:fontawesomefx-octicons:4.3.0-9.1.2")
   api("de.jensd:fontawesomefx-materialicons:2.2.0-9.1.2")
   api("de.jensd:fontawesomefx-materialdesignfont:2.0.26-9.1.2")
   api("de.jensd:fontawesomefx-fontawesome:4.7.0-9.1.2")
   api("com.jfoenix:jfoenix:9.0.10")
   api("org.apache.commons:commons-text:1.9")

   api("us.ihmc:jim3dsModelImporterJFX:0.7")
   api("us.ihmc:jimColModelImporterJFX:0.6")
   api("us.ihmc:jimFxmlModelImporterJFX:0.5")
   api("us.ihmc:jimObjModelImporterJFX:0.8")
   api("us.ihmc:jimStlMeshImporterJFX:0.7")
   api("us.ihmc:jimX3dModelImporterJFX:0.4")
}

testDependencies {
}
