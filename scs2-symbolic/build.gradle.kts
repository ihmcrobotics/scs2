plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

allprojects {
   tasks.javadoc {
      exclude("us/ihmc/**")
   }
}

mainDependencies {
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")

   api("us.ihmc:euclid:0.22.5")
   api("us.ihmc:euclid-frame:0.22.5")
   api("us.ihmc:ihmc-yovariables:0.13.7")
}

testDependencies {
}
