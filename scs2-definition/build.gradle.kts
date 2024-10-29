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
   api("us.ihmc:euclid:0.22.0")
   api("us.ihmc:euclid-shape:0.22.0")
   api("us.ihmc:euclid-frame:0.22.0")
   api("us.ihmc:ihmc-commons:0.34.0")
   api("us.ihmc:ihmc-yovariables:0.13.2")
   api("us.ihmc:mecano:17-0.19.0")
}

testDependencies {
}
