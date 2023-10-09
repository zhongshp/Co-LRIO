# CoLRIO

A ROS2 package of CoLRIO: LiDAR-Ranging-Inertial Centralized State Estimation for Robotic Swarms. 

https://github.com/PengYu-Team/Co-LRIO/assets/41199568/b5c13697-fddd-48bf-9f64-d3b8e1cb4978

## Prerequisites
  - [Ubuntu ROS2 Foxy](http://wiki.ros.org/ROS/Installation) (Robot Operating System 2 on Ubuntu 20.04)
  - CMake (Compilation Configuration Tool)
  - [PCL](https://pointclouds.org/downloads/linux.html) (Default Point Cloud Library on Ubuntu work normally)
  - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (Default Eigen library on Ubuntu work normally)
  - [GTSAM 4.2a8](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)

## Compilation
  Build CoLRIO:
  ```
  mkdir -p ~/cslam_ws/src
  cd ~/cslam_ws/src
  git clone https://github.com/PengYu-Team/Co-LRIO.git
  cd ../
  colcon build --symlink-install
  ```
## Run with Dataset
  - [our dataset] TBD.

  - [S3E dataset](https://github.com/PengYu-Team/S3E). The datasets are configured to run with default parameter.
  ```
  ros2 launch co_lrio run.launch.py
  ros2 bag play *your-bag-path*
  ```

## Acknowledgement

  - CoLRIO depends on [FAST-GICP](https://github.com/SMRT-AIST/fast_gicp) (Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, "Voxelized GICP for fast and accurate 3D point cloud registration".).

  - CoLRIO depends on [GncOptimizer](https://github.com/borglab/gtsam/blob/3a1fe574683f608759eaff4636ab53def600ce84/gtsam/nonlinear/GncOptimizer.h#L45) (Yang, Antonante, Tzoumas, Carlone, "Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection").
