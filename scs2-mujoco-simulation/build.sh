#!/bin/bash
set -e -o pipefail

usage() {
    echo "Usage: $0 {docker|install|wrap|clear}"
    echo "  docker   Build the Docker image used for the install/wrap steps."
    echo "  install  Stage MuJoCo SDK (headers + libmujoco.so) into build/ and resources/."
    echo "  wrap     Run JavaCPP to generate Mujoco.java and libjniMujoco.so. Requires 'install' first."
    echo "  clear    Remove the local install and build artifacts (does not touch the Java side)."
    exit 1
}

run_in_container() {
    docker run \
        --rm \
        --user "$(id -u):$(id -g)" \
        --volume "$(pwd):/home/robotlab/scs2-mujoco-simulation" \
        --workdir /home/robotlab/scs2-mujoco-simulation \
        ihmcrobotics/scs2-mujoco-simulation:0.1 bash "$1"
}

[ $# -eq 1 ] || usage

case "$1" in
    docker)
        docker build -t ihmcrobotics/scs2-mujoco-simulation:0.1 -f native-build/Dockerfile .
        ;;
    install)
        run_in_container native-build/install.sh
        ;;
    wrap)
        run_in_container native-build/wrap.sh
        ;;
    clear)
        rm -rf build install
        ;;
    *)
        usage
        ;;
esac
