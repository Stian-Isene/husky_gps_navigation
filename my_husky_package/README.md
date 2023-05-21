# my_husky_package
Project developed for navigating a clearpath husky using GPS coordinates.
This package is built alongside custom MBS software, and therefore calls some nodes/launch files that might be unavailable.



## Dependecies:
rosbridge_server is used for communicating with a websocket. http://wiki.ros.org/rosbridge_server

## Simulation:
 ``` roslaunch my_husky_package run_simulation.launch ```
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

This launches the MBS ROS package for navigation, gps_navigation in the mbs_gps_navigation package as well as several custom nodes.

### Nodes and launch files called by  custom_gps_navigation.launch
gps_to_transform_node: This is used for transforming coordinates from GPS to corresponding cartesian coordinates in the robot frame. A Robot Localization Navsat Transform node. THIS IS NOT WORKING AS INTENDED.

rosbridge_websocket: This is used for communicating with a websocket.

action_server: Action server responsible for handling user input during runtime.

intermediary_odometry_for_EKF_controll_node: Relay node for odometry sensor to the global Kalman filter. This is developed to be able to handle short-term loss in GPS correction. 

gps_to_cartesian_node: Service to transfrom GPS coordinates to cartesian coordinates. Uses gps_to_transform node for the transform. THIS IS NOT WORKING AS INTENDED

gps_user_service_server: Takes data from the websocket and saves to file to be used with the action server.



### Node explanation more in depth

#### action_server:
This is the main interaction during runtime, and where the user will send their commands.
To send a command to the action server:
Note: double press tab to autofill the structure of the message
``` rostopic pub /start_path/goal [tab][tab] ```

This message contains a single integer that represents different actions:
#### 1:  Move to UTM point
This reads the GPS points gained from the gps_user_service_server, converts them to UTM coordinates and sends them to sequentially to the /move_base action server.

#### 2: Follow Path Planner
This follows the path in the file "sweeping_path.txt". See https://github.com/Stian-Isene/sweeping_coverage_path_planner for the one made for this project.

#### 3: Save data to file
This converts the GPS points to UTM coordinates and saves it to a file. "cartesian_cordinates.txt"

#### 4: Move to GPS point
This reads the GPS points gained from the gps_user_service_server, transforms them to cartesian with the gps_to_cartesian_node service,  and sends the points sequentially to the /move_base action server.


#### intermediary_odometry_for_EKF_controll_node:
The philosophy behind this node is when there is no longer any GPS correction on the GPS signal from the gps driver, take the t-1 kalmanfiltered global position and add the difference between t and t-1 odometry from the husky_node. This will work short term. To implement this, change the global kalman filter input to the corresponding output of this node:
/odomety/gps -> /odometry/gps_filtered
/husky_velocity_controller/odom -> /husky_odom/filtered



