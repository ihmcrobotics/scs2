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
   api("us.ihmc:euclid:0.17.1")
   api("us.ihmc:euclid-shape:0.17.1")
   api("us.ihmc:euclid-frame:0.17.1")
   api("us.ihmc:ihmc-commons:0.30.5")
   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:mecano:0.9.0")
}

testDependencies {
}
