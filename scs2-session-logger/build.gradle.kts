plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-simulation:source") // TODO Need to fix this, it needs the Robot.

   api("us.ihmc:ihmc-robot-data-logger:0.28.8")
   api("com.github.luben:zstd-jni:1.5.5-10")
   api("org.antlr:antlr4-runtime:4.13.1")
   //api("org.lz4:lz4-java:1.8.0")
   api("com.github.vatbub:mslinks:1.0.6.2")
}

testDependencies {
   api("org.junit.jupiter:junit-jupiter-api:5.10.1")
}
