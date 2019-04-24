#!/bin/bash

sudo apt install libyaml-cpp-dev -y

cd /tmp
git clone https://github.com/AprilRobotics/apriltags.git apriltags
cd apriltags

# Checkout a known compatible and stable commit
git checkout fa3b50f3f1e4acaff0dd6e501ce9b5ca251dc3fd

# Use sed to modify the makefile to not build examples
sed -i 's/all: apriltag_demo opencv_demo/all: /' example/Makefile


# Install
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))
sudo make install