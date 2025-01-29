# Install with ROS


## Prerequisites
If you did not do it already, please install the following packages.

```bash
sudo apt-get update && apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev \
   libopencv-dev
``` 

## Get ROS
You can use this framework with the Robot Operating System ([ROS](https://www.ros.org/)) and you therefore first need to install it (Desktop-Full Install) by following the steps described in the [ROS Installation](https://wiki.ros.org/ROS/Installation).

## Gazebo
To install Gazebo checkout out their [documentation](https://gazebosim.org/tutorials?tut=ros_wrapper_versions).

Or in short

- ROS Melodic and newer: use Gazebo version 9.x `sudo apt-get install gazebo9`
- ROS Kinetic and newer: use Gazebo version 7.x `sudo apt-get install gazebo7`
- ROS Indigo: use Gazebo version 2.x `sudo apt-get install gazebo2`

## ROS Dependencies
Install system and ROS dependencies (on Ubuntu 20.04, replace `python-vcstool` with `python3-vcstool` ):

```
sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python-vcstool
```

Before continuing, make sure that your protobuf compiler version is 3.0.0. To check this out, type in a terminal `protoc --version`. If This is not the case, then check out [this guide](https://github.com/linux-on-ibm-z/docs/wiki/) on how to do it.

## Get catkin tools
Get catkin tools with the following commands:

```bash
sudo apt-get install python-pip 
sudo pip install catkin-tools
```

## Create a catkin workspace
Create a catkin workspace with the following commands by replacing `<ROS VERSION> `with the actual version of ROS you installed:

```bash
cd
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Install Flightmare
Clone the repository
```
cd ~/catkin_ws/src
git clone https://github.com/uzh-rpg/flightmare.git
```

Clone dependencies:
```
vcs-import < flightmare/flightros/dependencies.yaml
```

Build:
```
catkin build
```
Add sourcing of your catkin workspace and **FLIGHTMARE_PATH** environment variable to your `.bashrc` file:
```bash 
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export FLIGHTMARE_PATH=~/catkin_ws/src/flightmare" >> ~/.bashrc
source ~/.bashrc
```
Download Flightmare Unity Binary
Download the Flightmare Unity Binary **RPG_Flightmare.tar.xz** for rendering from the [Releases](https://github.com/uzh-rpg/flightmare/releases) and extract it into the */path/to/flightmare/flightrender*.

Now, you can move to [Basic Usage with ROS](use_ros.md) and run the example.

[Back to Main](wiki_home.md)