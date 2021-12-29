**This code was used to obtain the simulation examples presented in article entitled** *"A novel modeling approach to improve path-tracking of an articulated tractor-trailer system for precision agriculture.*

To run this examples you need to install:
1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
2. [MPCTools](https://bitbucket.org/rawlings-group/mpc-tools-casadi/src/master/)

If you want to use the HSL library, then you have to install the [HSL_MA97](https://www.hsl.rl.ac.uk/catalogue/hsl_ma97.html) from the HSL Mathematical Software Library. After installation, you might be able to uncomment line 116 from the following files: 
- export_articulated_tractor_trailer_model.py
- export_articulated_tractor_model.py

Then, you have to create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), clone the code provided in this repository in the `src` folder and then build the packages.

## Controlling the tractor's front block xy-position
To run simulation example shown in Section 5.1, you should open two terminal windows. In the first terminal, run:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_gazebo articulated_tractor_trailer_empty_low_resources.launch
```
This should open Gazebo simulator and you should see the articulated tractor-trailer model within this environment. 

Now, in order to start the NMPC controller, in the second terminal, run:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_trajectory articulated_tractor_trajectory.launch
```

The articulated tractor-trailer model shown within Gazebo should start moving. You should see the vehicle following a typical seeding path. In this simulation example, the model embedded within the NMPC controller does not know that the trailer is towed to the articulated tractor-trailer system.


## Controlling the trailer's xy-position
To run simulation example shown in Section 5.2, you should open two terminal windows. In the first terminal, run:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_gazebo articulated_tractor_trailer_empty_low_resources.launch
```
This should open Gazebo simulator and you should see the articulated tractor-trailer model within this environment. 

Now, in order to start the NMPC controller, in the second terminal, run:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_trajectory articulated_tractor_trailer_trajectory.launch
```
The articulated tractor-trailer model shown within Gazebo should start moving. You should see the vehicle following a typical seeding path. In this simulation example, the model embedded within the NMPC controller does know that the trailer is towed to the articulated tractor-trailer system.
