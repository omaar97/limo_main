# Pothole Detection during Autonomous Navigation:

A pothole detector during autonomous navigation using LIMO robot in Gazebo & ROS 2 Humble
This is the solution for the Robot Programming course assignment of the University of Lincoln MSc in RAS program by Omar Faris (Student ID# 28042200)

## Abstract:

This work presents a pothole detector for two simulated scenarios in Gazebo and ROS2 using AgileX LIMO robot. In the first scenario, the potholes in a simple world with purple potholes are detected by finding the contours of purple features in the robot RGB camera images. The second scenario involves finding potholes in a realistic world by identifying unique edges that the robot is observing. The robot navigates autonomously across the whole mapped world during the task regardless of its starting position by following waypoints in the map. A report about the potholes is generated after completing the task.

## Instructions for use:

0- Make sure [limo_ros2](https://github.com/LCAS/limo_ros2) package is built and sourced.

1- create a workspace and clone this repo:

> mkdir ros2_ws && cd ros2_ws && mkdir src && cd src && git clone https://github.com/omaar97/limo_main_OmarF.git

2- Build the package and source the directory:

> cd .. && colcon build && source install/setup.bash

3- Launch the Gazebo simulator (use potholes.world for the realistic world):

> ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_main/worlds/potholes_simple.world

4- In a new terminal, source the limo_ros2 package while also going to the directory of this package and sourcing it: 

> source limo_ros2/install/setup.bash && cd ros2_ws && source install/setup.bash

5- Launch Rviz with the nav2 parameters in the params directory:

> ros2 launch limo_navigation limo_navigation.launch.py use_sim_time:=true map:=src/limo_main/maps/potholes_20mm.yaml params_file:=src/limo_main/params/nav2_params.yaml

6- Check the variable self.map_type in detect_pothole node line 24. Its value should be 0 for the simple world and 1 for the realistic world. Don't forget to re-build the package if a different world is used!

7- Apply step 4 again and run the pothole detector node:

> ros2 run limo_main detect_pothole

8- Inside Rviz, add the MarkerArray display type to the screen and assign it the pothole_location topic.

9- Apply step 4 again and run the navigation node:

> ros2 run limo_main navigation_node

10- Wait for the robot to complete the task. Observe the potholes in Rviz and check the report after the navigation is completed!

## Important notes:

- The report is generated using reportlab package. Make sure it is installed.
- The navigation node uses the rotation shim controller, which is why the user should use the nav2 parameters yaml file in the params folder. 

## Limitations:

- Simple pothole detector: if two potholes are very close to each other, the robot may detect them as one pothole
- Realistic pothole detector: this detector is only accurate when the pothole is clearly in front of the robot. When the pothole is on the sides, the robot sometimes may not detect it. Another limitation is the fact that the method is based on thresholding, which could result in false negatives and false positives. The detector may also detect two potholes as one if they are very close to each other.
- Navigation strategy: it requires correct initial position of the robot in the map. Make sure to run the AMCL localizer if the robot does not know its initial location. Another limitation here is that the robot sometimes drifts and does not reach the intended position. When the error is accumulated, it can cause the robot to hit walls or skip some of the potholes if it does not see them.