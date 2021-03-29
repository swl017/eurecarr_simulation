# eurecarr_simulation

A simple ROS based car dynamics simulator.

## Install ROS

Follow either of the following.

### Official Documentation

`Melodic` with `Ubuntu 18.04` is recommended.\
Reference: http://wiki.ros.org/melodic/Installation/Ubuntu

### Convenience Install Script
Using scripts located at `etc/` of this repository, depending on you Ubuntu version, run

- Ubuntu 18.04
>> `sudo chmod 755 ./install_ros_melodic && bash ./install_ros_melodic.sh`
- Ubuntu 16.04
>> `sudo chmod 755 ./install_ros_kinetic && bash ./install_ros_kinetic.sh`

Note, aliases are setup in your `~/.bashrc`.\
To edit, run `gedit ~/.bashrc`.

## Download the Code

Clone this repository to you local machine.
```
cd ~/catkin_ws/src
git clone https://github.com/swl017/eurecarr_simulation.git
```

Finally, run the following command to install ROS dependencies.
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Install Python Dependencies

We need `pygame` to work with joysticks.
```
pip install pygame numpy
```
Note, ROS runs with `python 2.x`


## Running the Simulator

First, open a terminal and launch visualization.
```
roslaunch eurecarr_simulation simVis.launch
```

Then, in another terminal, run the node.
```
rosrun eurecarr_simulation simulate_dynamics.py
```

If the terminal cannot auto-complete the package(`eurecarr_simulation`), run the following and try again.
```
cd ~/catkin_ws
rospack profile
```

If everything is successfull, you should see the following.
- ROS node rviz
>> The vehicle is moved because of the initial speed in x direction.\
>> ![rviz](.etc/dynamics_sim_rviz.png)

- ROS node graph
>> Run the following command
>> ```
>> rqt_graph
>> ```
>> ![rqt_graph](.etc/dynamics_sim_rqt_graph.png)

Publish control(steering and throttle) value to `/control` topic to test your own self-driving algorithms.

## Unit Conventions
The following applies to `controlSubCallback` in `script/simulate_dynamics.py`
| variable name | meaning / use in the code | units |
| ---           | ---     | ---   |
| `msg.drive.steering_angle` | front wheel angle relative to the car heading | rad |
| `msg.drive.acceleration`   | throttle level | - |
| `self.inputs[0]`           | normalized steer input(min:-1, max:1) | - |
| `self.inputs[1]`           | normalized throttle input(min:-1, max:1) | - |
