# Basic Usage with ROS

## Launch Flightmare (use gazebo-based dynamics)
In this example, we show how to use the [RotorS](https://github.com/ethz-asl/rotors_simulator) for the quadrotor dynamics modelling, [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) for model-based controller, and **Flightmare** for image rendering.

```
roslaunch flightros rotors_gazebo.launch
```

We hope this example can serve as a starting point for many other applications. For example, Flightmare can be used with other multirotor models that comes with RotorS such as AscTec Hummingbird, the AscTec Pelican, or the AscTec Firefly. The default controller in [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control)  is a PID controller. Users have the option to use more advanced controller in this framework, such as [Perception-Aware Model Predictive Control](https://github.com/uzh-rpg/rpg_mpc).

[Back to Main](wiki_home.md)