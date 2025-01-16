plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:euclid-frame-shape:0.22.2")
   api("us.ihmc:ihmc-messager:0.2.1")
   api("us.ihmc:ihmc-yovariables:0.13.3")
   api("us.ihmc:mecano-yovariables:17-0.19.0")

   val bulletVersion = "3.25-1.5.11-ihmc-2"
   api("us.ihmc:bullet:$bulletVersion")
   api("us.ihmc:bullet:$bulletVersion:linux-x86_64")
   api("us.ihmc:bullet:$bulletVersion:windows-x86_64")
}

debugDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}

testDependencies {
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}
