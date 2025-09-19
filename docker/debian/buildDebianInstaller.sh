#!/bin/sh

docker build --tag ihmcrobotics/scs2-debian:0.1 .

docker run \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    --user $(id -u):$(id -g) \
    --volume $(pwd)/../..:/simulation-construction-set-2 \
    ihmcrobotics/scs2-debian:0.1 gradle buildDebianPackage
