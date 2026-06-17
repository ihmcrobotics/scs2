#!/bin/bash
set -e -o xtrace

# ==============================
# User Configuration Variables
# ==============================
MUJOCO_VERSION="3.2.7"
MUJOCO_TARBALL="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_TARBALL}"
LINUX_RESOURCES_DIR="src/main/resources/mujoco/linux-x86_64"
INSTALL_DIR="install/mujoco"

# ==============================
# Download MuJoCo SDK
# ==============================
mkdir -p build
cd build
if [ ! -f "${MUJOCO_TARBALL}" ]; then
    curl -L "${MUJOCO_URL}" -o "${MUJOCO_TARBALL}"
fi
rm -rf "mujoco-${MUJOCO_VERSION}"
tar -xzf "${MUJOCO_TARBALL}"
cd ..

# ==============================
# Stage Headers + Library
# ==============================
mkdir -p "${INSTALL_DIR}/include"
mkdir -p "${INSTALL_DIR}/lib"
cp -r "build/mujoco-${MUJOCO_VERSION}/include/mujoco" "${INSTALL_DIR}/include/"
cp -P "build/mujoco-${MUJOCO_VERSION}/lib/"libmujoco* "${INSTALL_DIR}/lib/"

# ==============================
# Copy Library to Resources
# ==============================
mkdir -p "${LINUX_RESOURCES_DIR}"
cp -P "${INSTALL_DIR}/lib/"libmujoco.so* "${LINUX_RESOURCES_DIR}/"

# Patch rpath so libjniMujoco.so finds libmujoco.so as a sibling at runtime.
for lib in "${LINUX_RESOURCES_DIR}"/*.so*; do
    if [ -f "$lib" ] && [ ! -L "$lib" ]; then
        patchelf --set-rpath '$ORIGIN' "$lib" || true
    fi
done
