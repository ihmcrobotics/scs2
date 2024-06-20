pluginManagement {
   plugins {
      id("us.ihmc.ihmc-build") version "0.29.6"
   }
}

buildscript {
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      maven { url = uri("https://mvnrepository.com/artifact/org.antlr/antlr4-runtime") }
      mavenLocal()
   }
   dependencies {
      classpath("us.ihmc:ihmc-build:0.29.6")
   }
}

val ihmcSettingsConfigurator = us.ihmc.build.IHMCSettingsConfigurator(settings, logger, extra)
ihmcSettingsConfigurator.checkRequiredPropertiesAreSet()
ihmcSettingsConfigurator.configureExtraSourceSets()
ihmcSettingsConfigurator.findAndIncludeCompositeBuilds()
