ROS
===

## Install ROS Fuerte
See ROS_Fuerte_Install.txt document.

## Install hector_quadrotor stack
```
rosws set hector http://tu-darmstadt-ros-pkg.googlecode.com/svn/branches/fuerte --svn
rosws update hector
. setup.bash
rosmake hector_quadrotor_demo
```

## Run hector_quadrotor_demo
```
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
rosrun hector_controller hector_control.py
rosrun hector_controller hector_teleop.py
```
