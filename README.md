# CoLRIO

A ROS2 package of CoLRIO: LiDAR-Ranging-Inertial Centralized State Estimation for Robotic Swarms. 

https://github.com/PengYu-Team/zhongshp/assets/41199568/81985d82-983c-4eca-898b-43e8f84e7b45

## Author
[Shipeng Zhong](https://github.com/zhongshp) & [Dapeng Feng](https://github.com/DapengFeng) & [Zhiqiang Chen](https://github.com/thisparticle)

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
  git clone https://github.com/zhongshp/Co-LRIO.git
  cd ../
  colcon build --symlink-install
  ```
## Run with Dataset
  - [S3E dataset](https://github.com/DapengFeng/S3E). The datasets are configured to run with default parameter.
  ```
  ros2 launch co_lrio run.launch.py
  ros2 bag play *your-bag-path*
  ```
  - [our dataset] please also found it in [S3E dataset](https://github.com/DapengFeng/S3E).
## Citation
This work is published in IEEE ICRA 2024 conference, and please cite related papers:

```
@INPROCEEDINGS{10611672,
  author={Zhong, Shipeng and Chen, Hongbo and Qi, Yuhua and Feng, Dapeng and Chen, Zhiqiang and Wu, Jin and Wen, Weisong and Liu, Ming},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={CoLRIO: LiDAR-Ranging-Inertial Centralized State Estimation for Robotic Swarms}, 
  year={2024},
  volume={},
  number={},
  pages={3920-3926},
  keywords={Simultaneous localization and mapping;Accuracy;Scalability;Collaboration;Computational efficiency;Sensors;Servers},
  doi={10.1109/ICRA57147.2024.10611672}}
```

```
@ARTICLE{10740801,
  author={Feng, Dapeng and Qi, Yuhua and Zhong, Shipeng and Chen, Zhiqiang and Chen, Qiming and Chen, Hongbo and Wu, Jin and Ma, Jun},
  journal={IEEE Robotics and Automation Letters}, 
  title={S3E: A Multi-Robot Multimodal Dataset for Collaborative SLAM}, 
  year={2024},
  volume={9},
  number={12},
  pages={11401-11408},
  keywords={Simultaneous localization and mapping;Robot sensing systems;Synchronization;Trajectory;Global navigation satellite system;Collaboration;Accuracy;Motion capture;Robot localization;Multi-robot systems;Multi-robot SLAM;data sets for SLAM;SLAM},
  doi={10.1109/LRA.2024.3490402}}
```

## Acknowledgement
  - We combined the front end of CoLRIO and the [DLO](https://github.com/vectr-ucla/direct_lidar_odometry) to achieve the 5th position in the [ICCV 2023 LiDAR-Inertial SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI).

  The Leaderboard is shown as follow:
  ![Leaderboard](https://github.com/PengYu-Team/Co-LRIO/assets/41199568/72168f1d-9c74-43d1-90ce-12383131f464)

  And the hardware and results are shown as follow:
  ![results table](https://github.com/PengYu-Team/Co-LRIO/assets/41199568/f75e8660-acd9-4961-8964-2e3edba1e965)
    
  - CoLRIO depends on [FAST-GICP](https://github.com/SMRT-AIST/fast_gicp) (Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, "Voxelized GICP for fast and accurate 3D point cloud registration".).

  - CoLRIO depends on [GncOptimizer](https://github.com/borglab/gtsam/blob/3a1fe574683f608759eaff4636ab53def600ce84/gtsam/nonlinear/GncOptimizer.h#L45) (Yang, Antonante, Tzoumas, Carlone, "Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection").
