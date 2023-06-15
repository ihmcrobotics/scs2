plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.0"
   id("us.ihmc.ihmc-cd") version "1.24"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")

   api("us.ihmc:ihmc-messager:0.2.0")
}

testDependencies {
}
