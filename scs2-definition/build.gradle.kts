plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ihmc-yovariables:0.11.0")
   api("us.ihmc:mecano:17-0.16.0")
}

testDependencies {
}
