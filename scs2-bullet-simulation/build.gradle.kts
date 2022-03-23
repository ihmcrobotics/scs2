plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:euclid:0.17.1")
   api("us.ihmc:euclid-shape:0.17.1")
   api("us.ihmc:euclid-frame:0.17.1")
   api("us.ihmc:euclid-frame-shape:0.17.1")
   api("us.ihmc:ihmc-messager:0.1.7")
   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:mecano-yovariables:0.9.1")

   api("us.ihmc:ihmc-javafx-toolkit:0.20.0") {
      exclude(group="us.ihmc", module="jassimp")
      exclude(group="us.ihmc", module="euclid")
      exclude(group="us.ihmc", module="euclid-shape")
      exclude(group="us.ihmc", module="euclid-frame")
   }

   val libGDXVersion = "1.10.0"
   api("com.badlogicgames.gdx:gdx-bullet:$libGDXVersion")
   api("com.badlogicgames.gdx:gdx-bullet-platform:$libGDXVersion:natives-desktop")

   val lwjglVersion = "3.2.3"
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-linux")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-windows")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-windows-x86")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-macos")
}

testDependencies {
}
