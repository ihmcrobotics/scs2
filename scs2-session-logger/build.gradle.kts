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
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:scs2-simulation:source") // TODO Need to fix this, it needs the Robot.

   api("us.ihmc:ihmc-robot-data-logger:0.39.4")
   api("us.ihmc:jros2-parser:1.5.1")
   api("org.antlr:antlr4-runtime:4.13.1")
   api("com.github.vatbub:mslinks:1.0.6.2")
   api("com.google.protobuf:protobuf-java:4.34.2")
   // Not using org.bytedeco:lz4-platform: it pulls in javacpp-platform, whose fixed javacpp:1.5.8
   // classifier list (incl. android-arm/x86, linux-armhf, linux/windows-x86 32-bit) gets bumped to
   // 1.5.11 by other native deps in this build (cuda/ffmpeg/openblas/opencv from ihmc-robot-data-logger),
   // and those obsolete 32-bit/Android classifiers were dropped at 1.5.11, breaking resolution.
   // Depending on the plain lz4 module plus only the native classifiers we actually avoid that.
   api("org.bytedeco:lz4:1.9.4-1.5.8")
   api("org.bytedeco:lz4:1.9.4-1.5.8:linux-x86_64")
   api("org.bytedeco:lz4:1.9.4-1.5.8:macosx-x86_64")
   api("org.bytedeco:lz4:1.9.4-1.5.8:macosx-arm64")
   api("org.bytedeco:lz4:1.9.4-1.5.8:windows-x86_64")
}

testDependencies {
   api("org.apache.commons:commons-math:2.2")
}
