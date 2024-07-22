#!/bin/bash

# This script is used to run the gradle scripts for deployment of the simulation-construction-set-2 project.

VERSION=$(gradle -p ../scs2-session-visualizer-jfx/ properties | grep "version:" | awk '{print $2}')

gradle -p ../scs2-session-visualizer-jfx/ clean
gradle -p ../scs2-session-visualizer-jfx/ buildDebianPackage
mv ../scs2-session-visualizer-jfx/deployment/debian/scs2-"$VERSION".deb .

gradle -p ../scs2-session-visualizer-jfx/ buildDebianPackage -Dihmc.build.javafxarm64=true
mv ../scs2-session-visualizer-jfx/deployment/debian/scs2-"$VERSION".deb scs2-"$VERSION"-aarch64.deb
