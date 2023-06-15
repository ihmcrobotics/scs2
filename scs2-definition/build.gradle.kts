plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.0"
   id("us.ihmc.ihmc-cd") version "1.24"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:euclid-shape:0.20.0")
   api("us.ihmc:euclid-frame:0.20.0")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ihmc-yovariables:0.9.18")
   api("us.ihmc:mecano:17-0.12.3")
}

testDependencies {
}
