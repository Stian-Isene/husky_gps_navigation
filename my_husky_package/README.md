# my_husky_package
Project developed for navigating a clearpath husky using GPS coordinates.
This package is built alongside custom MBS software, and therefore calls some nodes/launch files that might be unavailable.



## Dependecies:
rosbridge_server is used for communicating with a websocket. http://wiki.ros.org/rosbridge_server

## Simulation:
### ``` roslaunch my_husky_package run_simulation.launch ```
Launches the Gazebo simulation. This includes a world with real-life GPS-coordinates and corresponding backdrop, and spawns a robot.

There is also a way to start the world and spawn the husky separately. This is done to simplify the testing process.

``` roslaunch my_husky_package simulation_world.launch ```
Starts the world in Gazebo

``` roslaunch my_husky_package spawn_husky.launch ```
Spawns in the robot in Gazebo.


## GPS Navigation

It is assumed the user has MBS ROS packages for GPS navigation already installed.


To start the GPS navigation:
``` roslaunch my_husky_package custom_gps_navigation.launch ```

This starts launches the MBS ROS package for navigation, gps_navigation in the mbs_gps_navigation package as well as several custom nodes.

#### Nodes and launch files called by  custom_gps_navigation.launch
gps_to_transform_node: This is used for transforming coordinates from GPS to corresponding cartesian coordinates in the robot frame. THIS IS NOT WORKING AS INTENDED.

rosbridge_websocket: This is used for communicating with a websocket.




#### Node explanation
