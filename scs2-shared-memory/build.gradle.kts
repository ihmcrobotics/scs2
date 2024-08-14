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
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:ihmc-yovariables:0.12.2")
   api("us.hebi.matlab.mat:mfl-core:0.5.7")
}

testDependencies {
}
