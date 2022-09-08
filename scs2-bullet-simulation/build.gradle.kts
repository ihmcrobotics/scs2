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
   api("us.ihmc:euclid-frame-shape:0.19.0")
   api("us.ihmc:ihmc-messager:0.1.7")
   api("us.ihmc:ihmc-yovariables:0.9.16")
   api("us.ihmc:mecano-yovariables:0.11.2")

   val libGDXVersion = "1.11.0"
   api("com.badlogicgames.gdx:gdx-bullet:$libGDXVersion")
   api("com.badlogicgames.gdx:gdx-bullet-platform:$libGDXVersion:natives-desktop")
}

debugDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:17-0.21.2") {
      exclude(group="us.ihmc", module="jassimp")
      exclude(group="us.ihmc", module="euclid")
      exclude(group="us.ihmc", module="euclid-shape")
      exclude(group="us.ihmc", module="euclid-frame")
   }
}

testDependencies {
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}
