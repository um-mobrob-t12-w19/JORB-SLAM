#!/bin/bash

sudo apt install libgtk-3-dev -y
sudo apt install libglew-dev -y

cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build 
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTS=OFF \
         -DBUILD_EXAMPLES=OFF \
         -DBUILD_PANGOLIN_FFMPEG=OFF \
         -DBUILD_PANGOLIN_LIBJPEG=OFF \
         -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
         -DBUILD_PANGOLIN_LIBPNG=OFF \
         -DBUILD_PANGOLIN_LIBTIFF=OFF \
         -DBUILD_PANGOLIN_LIBUVC=OFF \
         -DBUILD_PANGOLIN_OPENNI=OFF \
         -DBUILD_PANGOLIN_OPENNI2=OFF \
         -DBUILD_PANGOLIN_PLEORA=OFF \
         -DBUILD_PANGOLIN_TELICAM=OFF \
         -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
         -DBUILD_PANGOLIN_V4L=OFF \
         -DBUILD_PANGOLIN_TOON=OFF \
         -DBUILD_PANGOLIN_VARS=ON \
         -DBUILD_PANGOLIN_VIDEO=OFF \
         -DBUILD_PANGOLIN_LIBDC1394=OFF \
         -DBUILD_PANGOLIN_DEPTHSENSE=OFF
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))
sudo make install
