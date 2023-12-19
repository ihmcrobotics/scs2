plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
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
}

testDependencies {
}
