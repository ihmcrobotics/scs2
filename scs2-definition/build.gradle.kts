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
   api("us.ihmc:euclid:0.19.0")
   api("us.ihmc:euclid-shape:0.19.0")
   api("us.ihmc:euclid-frame:0.19.0")
   api("us.ihmc:ihmc-commons:0.31.0")
   api("us.ihmc:ihmc-yovariables:0.9.16")
   api("us.ihmc:mecano:17-0.11.3")
}

testDependencies {
}
