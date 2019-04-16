# A Robust Vision-Based Model Predictive Control Package #

<img src="src/assimp_loader/assets/zeppelin/zeppelin_gtaV_mod.jpg" alt="" width="100%;">

This software has been tested using Ubuntu 18.04 and Ros Melodic.

## Prerequisite: ACADO toolkit ##

```bash
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
echo "source <PATH_TO_ACADO_ROOT>/ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```
where <ACADO_ROOT> is the absolute patg to the ACADOtoolkit folder.

## Installation ##

```bash
sudo apt-get install libyaml-cpp-dev libglfw3-dev python-catkin-tools ros-melodic-octomap-ros libgoogle-glog-dev libglew-dev libglm-dev
## Creating the workspace 
mkdir -p ~/rvb_mpc_ws/src
cd ~/rvb_mpc_ws/
catkin_init_workspace
cd src/ 
git clone https://github.com/cirpote/rvb_mpc && cd rvb_mpc
sh clone_dependencies.sh
catkin_build
```

## Usage ##

```bash
roslaunch rvb_mpc random_flight.launch
```

args:
 - gui: Activate the GAZEBO gui (default = false)
 - random_spawn_dyn_obj: Activate the random dynamic object spawing along the planned traj (default = false)
 - random_waypoint_generation: Activate the random waypoint generation (default = false)
 - 
 Example: 
```bash
roslaunch rvb_mpc random_flight.launch gui:=true
```
