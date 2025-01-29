# Install with pip

# Prerequisites
If you did not do it already, please install the following packages.

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev \
   libopencv-dev 
```

# Python environment
It is a good idea to use virtual environments (virtualenvs) or Anaconda to make sure packages from different projects do not interfere with each other. Check here for Anaconda installation.

To create an environment with python3.7

```bash
conda create --name v python=3.7.16
```
Activate a named Conda environment
```bash
conda activate v
```

# Install Flightmare
Clone the project to your desktop (or any other directory)

```bash
cd ~/Desktop
git clone https://github.com/AvidEslami/flightmare.git
```

# Add Environment Variable
Add FLIGHTMARE_PATH environment variable to your .bashrc file:

```bash
echo "export FLIGHTMARE_PATH=~/Desktop/flightmare" >> ~/.bashrc
source ~/.bashrc
```

# Install dependencies
Here, we need to install specific versions of packages, with the full list [here](package_list.md). 
```bash
conda activate v
cd flightmare/
pip install scikit-build ruamel.yaml==0.17.33 protobuf==3.20.0
```

Install tensorflow GPU (for non-gpu user, use pip install tensorflow==1.14)
```bash
pip install tensorflow-gpu==1.14 
```

# Install flighmare (flightlib)
```bash
cd flightmare/flightlib
# it first compile the flightlib and then install it as a python package.
pip install .
```

After installing flightlib, you can following the [Basic Usage with Python](use_python.md) for some Reinforcement learning examples.



[Back to Main](wiki_home.md)