plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications()
}

allprojects {
   tasks.javadoc {
      exclude("us/ihmc/**")
   }
}

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:euclid-frame-shape:0.22.5")
   api("us.ihmc:ihmc-messager:0.2.1")
   api("us.ihmc:ihmc-yovariables:0.13.7")
   api("us.ihmc:mecano-yovariables:17-0.19.3")

   api("org.bytedeco:javacpp:1.5.11")
   api("us.ihmc:ihmc-native-library-loader:2.0.6")
}

debugDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:scs2-session-visualizer-jfx:source")
   // SimulationConstructionSet2: user-facing wrapper around SimulationSession + visualizer; needed
   // by PushRobotControllerSCS2.addPushButtonToSCS to install the toolbar button.
   api("us.ihmc:scs2-simulation-construction-set:source")
   // Canonical SCS2 push tool. Lives in ihmc-open-robotics-software for historical reasons; the
   // engine-agnostic ExternalWrenchPoint plumbing in scs2-simulation makes it work against any
   // PhysicsEngine that forwards external wrenches (which our MujocoPhysicsEngine does).
   api("us.ihmc:ihmc-simulation-toolkit:source")
}

testDependencies {
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}
