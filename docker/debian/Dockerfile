# Built on the oldest supported Ubuntu release: jpackage records the build host's package names and glibc requirements in the .deb, so a
# .deb built here also installs on newer releases (the reverse is not true, e.g. 24.04 renamed libasound2 to libasound2t64).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    fakeroot \
    curl \
    zip \
    unzip \
    openjdk-17-jdk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sL https://services.gradle.org/distributions/gradle-8.14-bin.zip -o gradle.zip \
  && unzip -q gradle.zip -d gradle \
  && mv gradle/gradle-8.14/ /opt/. \
  && ln -s /opt/gradle-8.14/ /opt/gradle \
  && ln -s /opt/gradle/bin/gradle /usr/bin/gradle \
  && rm -rf gradle.zip gradle/

WORKDIR /simulation-construction-set-2/scs2-session-visualizer-jfx
