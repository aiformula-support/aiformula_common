# EC7D_AIformula_Control

ROS2 packages for AI Formula developper.
ROS2 Foxy Fitzroy (Ubuntu 20.04)

## About AI Formula
AI Formula is a technical challenge where robot car drives autonomously in mission course. 
In purpose, it can acquire the necessary technology for next-generation mobility research. through, autonomous racing competing speed and intelligence in the real world.\
This package is the foundation of the AIFormula system.

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

## Getiing Start
To start all node of aiformula:\
**Note:** please need to conect all interfaces
```
ros2 launch launchers all_node.launch
```
## install
Install aiformula package and build :\
**Note** This package contains submodules, if the build of a submodule fails, please refer to original packages.
```
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone  --recursive https://github.com/honda-hgrx-idcs/EC7D_AIformula_Control.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
```
## docker
To start docker: \
```
cd ~/ros2_ws/src/EC7D_AIformula_Control/docker
./docker_build_aiformula_foxy_amd.sh
./docker_run_aiformula_foxy_amd.sh
```
