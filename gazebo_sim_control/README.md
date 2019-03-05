# Gazebo Sim Control

This package should provide a set of plugins to have a better control over
Gazebo simulations.


## Build and Run

The code is a catkin package, therefore the standard procedure (`catkin build`)
should suffice.

An example is available that loads the provided plugins. Type:
```
roslaunch gazebo_sim_control example.launch
```
Note that the simulators starts in a paused state.


## Simulation Step

The plugin `gzsim_control_step` provides a simple ROS interface to perform
a certain number of iterations when the simulation is paused. You need to load
the plugin in your world file (see the example `worlds/empty_world.sdf`). In
addition, gazebo must be started with the ROS connection. Use the package
`gazebo_ros` for this purpose, and have a look at `launch/empty_world.launch`.

When Gazebo starts, you should see the message
```
SimulationStep: plugin started in ROS namespace gazebo_sim_control
```
or similar, meaning that the plugin loaded successfully. After that, you should
be able to pause the simulation and to publish on the topic
`gazebo_sim_control/sim_step` to advance the simulation manually.



## TF publisher

`gzsim_control_tfpub` is a plugin that allows to publish frame transformations
over `/tf`. To use it, simply attach it to an existing model, and it will
publish all links transformations in the model.

Have a look at the example world to see available parameters and the source code
for more information.


