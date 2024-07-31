plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

val javaCPPVersion = "1.5.9"

mainDependencies {
   api("us.ihmc:scs2-simulation:source")
   api("us.ihmc:scs2-definition:source")
   api("us.ihmc:scs2-shared-memory:source")
   api("us.ihmc:scs2-session:source")
   api("us.ihmc:euclid-frame-shape:0.21.0")
   api("us.ihmc:ihmc-messager:0.2.0")
   api("us.ihmc:ihmc-yovariables:0.12.0")
   api("us.ihmc:mecano-yovariables:17-0.18.1")

   apiBytedecoNatives("javacpp")
   apiBytedecoNatives("bullet", "3.25-")
}

debugDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}

testDependencies {
   api("us.ihmc:scs2-session-visualizer-jfx:source")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoNatives(name: String, versionPrefix: String = "")
{
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:linux-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:windows-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoSelective(dependencyNotation: String)
{
   api(dependencyNotation) {
      exclude(group = "org.bytedeco")
   }
}
