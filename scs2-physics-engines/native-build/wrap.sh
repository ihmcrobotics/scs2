#!/bin/bash
set -e -o xtrace

# ==============================
# User Configuration Variables
# ==============================
JAVACPP_VERSION="1.5.11"
LINUX_RESOURCES_DIR="src/main/resources/mujoco/linux-x86_64"
GENERATED_JAVA_DIR="src/main/generated-java/us/ihmc/scs2/simulation/mujoco"
CXX_RELEASE_FLAGS="-O3 -fPIC -std=c++17"

# ==============================
# Stage Java Sources for JavaCPP
# ==============================
# JavaCPP needs the preset class on the classpath and a writable build dir.
mkdir -p build/java
cp -r src/main/java/* build/java/

cd build/java

if [ ! -f javacpp.jar ]; then
    curl -L "https://github.com/bytedeco/javacpp/releases/download/${JAVACPP_VERSION}/javacpp-platform-${JAVACPP_VERSION}-bin.zip" -o javacpp.zip
    unzip -j javacpp.zip
fi

# ==============================
# Step 1: Parse the preset to emit Mujoco.java
# ==============================
java -jar javacpp.jar us/ihmc/scs2/simulation/mujoco/preset/MujocoInfoMapper.java

# ==============================
# Step 2: Compile the JNI shim into libjniMujoco.so
# ==============================
java -jar javacpp.jar us/ihmc/scs2/simulation/mujoco/Mujoco.java \
    -d "../../${LINUX_RESOURCES_DIR}" \
    -Dplatform.compiler="clang++" \
    -Dplatform.compiler.default="${CXX_RELEASE_FLAGS}"

cd ../..

# ==============================
# Copy Generated Java to generated-java sourceset
# ==============================
mkdir -p "${GENERATED_JAVA_DIR}"
rm -f "${GENERATED_JAVA_DIR}/Mujoco.java"
cp build/java/us/ihmc/scs2/simulation/mujoco/Mujoco.java "${GENERATED_JAVA_DIR}/"

# ==============================
# Patch rpath on the produced JNI library
# ==============================
patchelf --set-rpath '$ORIGIN' "${LINUX_RESOURCES_DIR}/libjniMujoco.so" || true
