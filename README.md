# EC7D_AIformula_Control
AI Formula is a technical challenge in which robot cars drive autonomously on a race course given a mission. Through competing for speed and intelligence in a real-world environment, AI Formula will provide an opportunity for rising engineers to acquire the skills and technology necessary for next-generation mobility research. This repository is the foundation of the AIFormula system.

![AIFormula_run](https://github.com/honda-hgrx-idcs/EC7D_AIformula_Control/assets/113084733/df02c1ec-0556-4c77-a834-ebc2fe192ac5)

Functions to be provided in this package:
* camera data
* imu data
* motor controller
* vehicle
* simulator
* perception (coming soon)
* planning   (coming soon)
* control    (coming soon)

## Dependencies
* ROS2 Foxy (Ubuntu 20.04)
* [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
* [VectorNav](https://github.com/dawonn/vectornav)

## Getting Started

### Installation

* **Local Environment:**\
Clone this repository and build:\
**Note:** This package contains submodules. If the build of a submodule fails, please refer to the original packages (linked above).
  ```bash
  mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
  cd ~/ros2_ws/src/ #use your current ros2 workspace folder
  git clone --recursive https://github.com/honda-hgrx-idcs/EC7D_AIformula_Control.git
  sed -i 's/tf2_geometry_msgs\.hpp/tf2_geometry_msgs.h/g' ~/ros2_ws/src//EC7D_AIformula_Control/sensing/vectornav/vectornav/src/vn_sensor_msgs.cc
  cd ..
  sudo apt update
  rosdep install --from-paths src --ignore-src -r -y # install dependencies
  colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
  ```

* **Docker Environment:**\
To start docker:
  ```bash
  cd ~/ros2_ws/src/EC7D_AIformula_Control/docker
  ./docker_build_aiformula_foxy_amd.sh
  ./docker_run_aiformula_foxy_amd.sh
  ```

### Running the Example
To start all nodes of aiformula:\
**Note:** This command launches the following nodes: camera data, imu data, can data, motor controller, tf, joy.
```bash
ros2 launch launchers all_nodes.launch.py
```
