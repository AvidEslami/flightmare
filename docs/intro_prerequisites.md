# Prerequisites

## Git
For the following installation instructions you will need git which you can set up with the following commands:

```bash 
sudo apt-get install git
git config --global user.name "Your Name Here"
git config --global user.email "Same Email as used for github"
git config --global color.ui true
```

## Packages
Flightmare requires CMake and GCC compiler. You will also need system packages python3, OpenMPI, and OpenCV.

```bash 
apt-get update && apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev \
   libopencv-dev 
```

## Python
- [Install with pip](install_pip.md). (Recommend for RL tasks)

## ROS
- [Install with ROS](install_ros.md) (Recommend for other tasks)

[Back to Main](wiki_home.md)