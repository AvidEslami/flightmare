# Basic Usage with Python

## Install the Flightmare library flightlib
To run the reinforcement learning example, you have to first install flightmare library please follow the guide of [Install with pip](install_pip.md).

## Install the Flightmare RL baselines flightrl
```bash
conda activate v
cd /path/to/flightmare/flightrl
pip install -e.
```

### Train neural network controller using PPO
``` bash
cd examples
python3 run_drone_control.py --train 1
```

## Test a pre-trained neural network controller
```bash
cd examples
python3 run_drone_control.py --train 0
```

## With Unity Rendering
To use unity rendering, you need first download the binary from **Releases** and extract it into the **flightrender** folder. To enable unity for visualization, double click the extracted executable file **RPG_Flightmare.x84-64** and then test a pre-trained controller
```bash
cd examples
python3 run_drone_control.py --train 0 --render 1
```

[Back to Main](wiki_home.md)