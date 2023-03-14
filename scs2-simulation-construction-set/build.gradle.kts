plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("com.github.johnrengelman.shadow") version "7.1.2"
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
   api("us.ihmc:scs2-session-visualizer-jfx:source")
   api("us.ihmc:ihmc-ros2-library:0.22.2")
   api("org.ros.rosjava_messages:std_msgs:0.5.10")
   
   api("us.ihmc:ros2-common-interfaces:0.22.2") {
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-api")
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-engine")
      exclude(group = "org.junit.platform", module = "junit-platform-commons")
      exclude(group = "org.junit.platform", module = "junit-platform-launcher")
   }
}

testDependencies {
}


// Applies to the shadow plugin; i.e. installShadowDist
application.mainClass.set("us.ihmc.scs2.ros.Ros2Visualizer")

tasks {
   named<com.github.jengelman.gradle.plugins.shadow.tasks.ShadowJar>("shadowJar") {
      setZip64(true)
      //minimize()
      transform(com.github.jengelman.gradle.plugins.shadow.transformers.Log4j2PluginsCacheFileTransformer::class.java)
   }
}
