Es 1 - Display Ros turtle on Rviz

- roslaunch esercizi esercizio1.launch
- open rviz
- add topic type "Pose" on Rviz and select "rviz_pose"
- move the turtle with the arrows

Es 2 - Controller

- roslaunch esercizi esercizio2.launch 
- if you want to saturate the control output add: linear_sat:="'NUMBER'" angular_sat:="'NUMBER'" ,with 
  NUMBER > 0 (example: roslaunch esercizi esercizio2.launch linear_sat:="'2'" angular_sat:="'3'" ).
  If NUMBER < 0 no saturation will be applied. The default value for both the saturations is -1, that 
  means no saturations. The controller is going to follow the reference values of linear and angular 
  velocity set in the launch file (linear = 1, angular = 1).
- open rqt_plot and display /turtle1/pose (select angular and linear velocities) and /vel_requested

Es 3 - Bound turtle movements in the upper part of the window

- roslaunch esercizi esercizio3.launch

Es 4 - Gather 4 turtles in the centroid one by one

- roslaunch esercizi esercizio4.launch

Es 5 - Collision Avoidance

- roslaunch esercizi esercizio5.launch x_goal:="'NUMBER'" y_goal:="'NUMBER'" 
  If the arguments are not specified the default values for goal point coordinates are (x,y)=(10,4).

	
