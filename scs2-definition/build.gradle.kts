plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.2")
   api("us.ihmc:euclid-shape:0.22.2")
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:ihmc-commons:0.34.0")
   api("us.ihmc:ihmc-yovariables:0.13.3")
   api("us.ihmc:mecano:17-0.19.0")
}

testDependencies {
}
