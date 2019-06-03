# ROS-Turtle-exercises

This ROS package includes the resolution of five exercises, developed on ROS Kinetic, to get familiar with ROS.

## Ex. 1 - Display Turtlebot on Rviz
Visualize Turtlebot on Rviz and control it with keyboard arrows.

- roslaunch esercizi esercizio1.launch
- open rviz
- add topic type "Pose" on Rviz and select "rviz_pose"
- move the turtle with the arrows

## Ex. 2 - Turtlebot speed controller
Write a node simulating a speed controller, taking as input a custom message (linear and angular speed) and giving a saturated control action as output. Saturation values must be set as node parameter. 

- roslaunch esercizi esercizio2.launch 
- if you want to saturate the control output add: linear_sat:="'NUMBER'" angular_sat:="'NUMBER'" ,with 
  NUMBER > 0 (example: roslaunch esercizi esercizio2.launch linear_sat:="'2'" angular_sat:="'3'" ).
  If NUMBER < 0 no saturation will be applied. The default value for both the saturations is -1, that 
  means no saturation. The controller is going to follow the reference values of linear and angular 
  velocity set in the launch file (linear = 1, angular = 1).
- open rqt_plot and display /turtle1/pose (select angular and linear velocities) and /vel_requested

## Ex. 3 - Turtlebot Bounding
Bound turtle movements in the upper part of the window

- roslaunch esercizi esercizio3.launch

## Ex. 4 - Turtles Gathering
Generate 4 turtles on the map and control them one by one on the centroid of the initial positions. Before leaving every turtle must wait the previous turtle to arrive to the centroid.

- roslaunch esercizi esercizio4.launch

## EX. 5 - Collision Avoidance
Generate 3 turtles on the map and control one of them to a selected point. During the motion this turtle must avoid the other 2 turtles.

- roslaunch esercizi esercizio5.launch x_goal:="'NUMBER'" y_goal:="'NUMBER'" 
  If the arguments are not specified the default values for goal point coordinates are (x,y)=(10,4).

	
