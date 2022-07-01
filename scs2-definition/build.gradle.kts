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
   api("us.ihmc:euclid:0.18.1")
   api("us.ihmc:euclid-shape:0.18.1")
   api("us.ihmc:euclid-frame:0.18.1")
   api("us.ihmc:ihmc-commons:0.31.0")
   api("us.ihmc:ihmc-yovariables:0.9.15")
   api("us.ihmc:mecano:0.11.2")
}

testDependencies {
}
