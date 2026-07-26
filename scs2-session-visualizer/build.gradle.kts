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

   var javaFXVersion = "17.0.20"
   api(ihmc.javaFXModule("base", javaFXVersion)) // This is for using the property data structure. Not sure if that's the best thing to do.
}

testDependencies {
}
