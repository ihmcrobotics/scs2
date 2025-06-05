plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.5")
   api("us.ihmc:euclid-shape:0.22.3")
   api("us.ihmc:euclid-frame:0.22.3")
   api("us.ihmc:ihmc-commons:0.35.1")
   api("us.ihmc:ihmc-yovariables:0.13.6")
   api("us.ihmc:mecano:17-0.19.2")
}

testDependencies {
}
