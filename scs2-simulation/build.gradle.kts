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
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:euclid:0.17.2")
   api("us.ihmc:euclid-shape:0.17.2")
   api("us.ihmc:euclid-frame:0.17.2")
   api("us.ihmc:euclid-frame-shape:0.17.2")
   api("us.ihmc:ihmc-messager:0.1.7")
   api("us.ihmc:mecano-yovariables:0.11.1")
}

testDependencies {
}
