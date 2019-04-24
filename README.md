# JORB-SLAM
JORB-SLAM is a multi-agent SLAM system based on ORB-SLAM2. Authors are **Martin D Deegan, Yuanxin Zhong, Purva Kulkarni, Christine Searle, Kaustav Chakraborty**

## ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. Please check [here for the Original Implementation](https://github.com/raulmur/ORB_SLAM2)

## Related Publications:

Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

Wang, John, and Edwin Olson. **AprilTag 2: Efficient and robust fiducial detection.** 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2016. **[PDF](https://april.eecs.umich.edu/pdfs/wang2016iros.pdf)**.

# Build

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Before running the build script, please turn to `install_apriltags.sh` and `install_pangolin.sh` for **Pangolin** and **AprilTags** installation, which are two dependencies of this program:
```
cd ORB_SLAM2

./install_apriltags.sh
./install_pangolin.sh

chmod +x build.sh
./build.sh
```

If you don't run `install_apriltags.sh`, you need to install `libyaml-cpp` with 
```
sudo apt install libyaml-cpp-dev
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **rgbd_tum** and **stereo_kitti** in *Examples* folder.

# Running Examples

Note: Follow the instructions in the command line when running.

## (Stereo) KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

### Sample data sequence
We included a sample data sequence out of box under folder `datasets`, this sample is cut out from sequence 00 of KITTI odometry dataset. Unzip file `kitti_odometry_00.zip` into `dataset/kitti_odometry_00` and execute `run_kitti.sh`, then you can find the system running on the sample sequence.

## (RGBD) EECS Building Dataset

1. Download a sequence from our [EECS Building Dataset](https://drive.google.com/drive/folders/1gWPB1DK2V4TwV0HjROTEs3wOZjfg8BZU?usp=sharing) collected using RGB-D cameras and uncompress it. Or you can use the sample data sequence provided at the folder `datasets`.

2. Unzip file `EECSBlue$.zip` into `dataset/EECSBlue`, `EECSOrange$.zip` into `dataset/EECSOrange` (`$` can be a number or nothing).

3. Execute `run_eecs.sh` then you can find the system running on the sample sequence. The final cloud view shows convisibility constraints in green lines and April tags constraint in red lines.

## Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV, and you can do calibration with ROS calibration package or MATLAB Calibration App. RGB-D input must be synchronized and depth registered. Specifically for Intel Realsense Camera data bag, we provide [Python scripts](https://github.com/um-mobrob-t12-w19/ORB_SLAM2/tree/master/Examples/RGB-D/utils) to convert the data bag to TUM dataset format.

# SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

