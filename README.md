# A Robust Vision-Based Model Predictive Control Package #

<img src="src/assimp_loader/assets/zeppelin/zeppelin_gtaV_mod.jpg" alt="" width="100%;">

This software has been tested using Ubuntu 18.04 and Ros Melodic.

## Installation ##

```bash
sudo apt-get install libyaml-cpp-dev libglfw3-dev python-catkin-tools
## Creating the workspace 
mkdir -p ~/rvb_mpc_ws/src
cd ~/rvb_mpc_ws/
catkin init workspace
cd src/ 
git clone https://github.com/cirpote/rvb_mpc && cd rvb_mpc
sh clone_dependencies.sh
catkin_build
```
