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
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-simulation:source") // TODO Need to fix this, it needs the Robot.

   api("us.ihmc:ihmc-robot-data-logger:0.28.5")
}

testDependencies {
}
