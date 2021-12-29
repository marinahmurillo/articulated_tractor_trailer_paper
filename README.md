**This code was used to obtain the simulation examples presented in article entitled** *"A novel modeling approach to improve path-tracking of an articulated tractor-trailer system for precision agriculture.*

To run this examples you need to install:
1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
2. [MPCTools](https://bitbucket.org/rawlings-group/mpc-tools-casadi/src/master/)

If you want to use the HSL library, then you have to install the [HSL_MA97](https://www.hsl.rl.ac.uk/catalogue/hsl_ma97.html) from the HSL Mathematical Software Library. After installation, you might be able to uncomment line 116 from `export_articulated_tractor_trailer_model.py` and `export_articulated_tractor_model.py`.

Then, you have to create a `catkin workspace`, clone the code provided in this repository in the `src` folder and build the packages provided.

To run the examples controlling the trailer xy-position, in one terminal window run the following:

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_gazebo articulated_tractor_trailer_empty_low_resources.launch
```

In a new terminal, run 
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_trajectory articulated_tractor_trailer_trajectory.launch
```

You should see the articulated tractor-trailer system model within Gazebo simulator following a typical seeding path. In this case, the model embedded within the controller does know about the trailer kinematics.

If you want to run the examples controlling the front block of the tractor xy-position, you should run following:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch articulated_tractor_trailer_trajectory articulated_tractor_trajectory.launch
```

You should see the articulated tractor-trailer system model within Gazebo simulator following a typical seeding path. In this case, the model embedded within the controller does not know about the trailer kinematics.
