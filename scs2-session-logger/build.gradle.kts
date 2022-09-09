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
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-simulation:source") // TODO Need to fix this, it needs the Robot.

   api("us.ihmc:ihmc-robot-data-logger:17-0.23.4")
}

testDependencies {
}
