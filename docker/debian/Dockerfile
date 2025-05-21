FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
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
