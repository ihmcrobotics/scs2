plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:euclid-frame:0.19.1")
   api("us.ihmc:ihmc-yovariables:0.9.16")
   api("us.hebi.matlab.mat:mfl-core:0.5.7")
}

testDependencies {
}
