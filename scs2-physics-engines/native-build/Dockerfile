FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    build-essential \
    cmake \
    clang \
    git \
    curl \
    unzip \
    patchelf \
    openjdk-17-jdk \
    > /dev/null

WORKDIR /home/robotlab/scs2-mujoco-simulation
