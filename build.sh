echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))

cd ../../Pangolin/

echo "Configuring and building Thirdparty/Pangolin ..."

mkdir build
cd build
cmake .. -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make stereo_kitti -j$((num_procs_avail > 1 ? num_procs_avail : 1))
cd ..

echo "Converting vocabulary to binary"
./build/binarize
