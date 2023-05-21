#!/usr/bin/env python3
import copy

import rospy
import rospkg
import actionlib
import os
import utm
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from my_husky_messages.msg import MovePathActionGoal, MovePathActionResult, MovePathActionFeedback, MovePathAction
from my_husky_messages.srv import GPSToCartesian, GPSToCartesianRequest, GPSToCartesianResponse
from std_msgs.msg import String



class ActionServer:
    #Creating the necessary variables for the action server
    _feedback = MovePathActionFeedback
    _result = MovePathActionResult

    def __init__(self):
        #INitializes the action server
        self.server = actionlib.SimpleActionServer('start_path', MovePathAction, execute_cb=self.start_action, auto_start=False)

        #Initializes a publisher for real time debugging
        self.pub_bug_talker = rospy.Publisher('bug_reporter', String, queue_size=10)

        #Starts the action server
        self.server.start()
        rospy.loginfo("Server is up and running")


    def start_action(self, goal: MovePathActionGoal):
        #This function starts when the action server is being called
        #In the goal there is an int, this will be used for controll.

        # Goal = 1: Move to the UTM coordinates from the GPS points found in the file
        # Goal = 2: Move the path found in the pathplanner file
        # Goal = 3: Converts the GPS coordinates to UTM coordinates, and save it to file
        # Goal = 4: Move to GPS coordinates transformed to Cartesian coordinates.

        #Gets the GPS data from a file
        package_name = 'gps_user_service'
        file_name = '/gps_coordinates.txt'
        self.gps_points_list = self.get_data_from_file(package_name, file_name)

        #Publishes the number of gps points found in the file.
        self.pub_bug_talker.publish(f"The list of gps points contains {len(self.gps_points_list)} entries")

        #Does the action based on the goal
        if goal.input == 1:
            self.move_to_utm_points()
        elif goal.input == 2:
            self.follow_path_planner()
        elif goal.input == 3:
            self.save_data_to_file()
        elif goal.input == 4:
            self.move_to_gps_points_using_cartesian()
        else:
            rospy.loginfo("Action not found")

        #Finishes the action
        self.server.set_succeeded(self._result) #Sets the result of the action.


    """
    Moving to a point in the UTM coordinate frame
    
    Converts the GPS points to UTM points
    Saves in a list as MoveBaseGoal
    Sends them to the movebase_client
    """
    def move_to_utm_points(self):

        #Creates the list to store the goals
        move_base_goal_list = list()

        #Creating the template of the MoveBaseGoals
        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = "utm"
        #Initialises the orientation, this needs to be changed
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1


        #Creates the MoveBaseGoal from the GPS Points
        for i in range(len(self.gps_points_list)):


            lat = self.gps_points_list[i][0]
            lon = self.gps_points_list[i][1]

            #Converts from GPS to UTM
            temp = utm.from_latlon(lat,lon)

            #Creates a new instance of the ActionBaseGoal class
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            #Fills it with corresponding values
            move_base_goal_temp.target_pose.pose.position.x = temp[0]
            move_base_goal_temp.target_pose.pose.position.y = temp[1]

            #Appends the list
            move_base_goal_list.append(move_base_goal_temp)


        rospy.loginfo("Starting to move")
        #Sends the list of the ActionBaseGoals to the movebase_client for sequential navigation
        self.movebase_client(move_base_goal_list)

    """
        Moves sequentially to points using GPS points transformed to Cartesian points
        
        Transform from GPS to Cartesian using a ROS Service
        
        Saves them to a list of MoveBaseGoal
        
        Sends them to a movebase_client for sequential navigation
    """
    def move_to_gps_points_using_cartesian(self):
        #Initializes the connection to the transform service
        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)

        move_base_goal_list = list()

        #Creating the template of the MoveBaseGoals
        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = "map"
        #Initialises the orientation, this needs to be changed
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1



        for i in range(len(self.gps_points_list)):
            rospy.loginfo(f"Latitude: {self.gps_points_list[i][0]}")
            rospy.loginfo(f"Longitude: {self.gps_points_list[i][1]}")

            #Sends the GPS points and recieves the cartesian points
            respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
            cartesian_coordinates.wait_for_service()
            rospy.loginfo(f"x = {respond.x_axis}")
            rospy.loginfo(f"y = {respond.y_axis}")

            # Copies the object, so we can use the same template
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = respond.x_axis
            move_base_goal_temp.target_pose.pose.position.x = respond.y_axis

            #Appends the list of goals
            move_base_goal_list.append(move_base_goal_temp)



        rospy.loginfo("Starting to move")
        #Sends the list of the ActionBaseGoals to the movebase_client for sequential navigation
        self.movebase_client(move_base_goal_list)

    """
        Follows the path in sweeping_path.txt
        
        Gets the path from sweeping_path.txt
        
        Creates the MoveBaseActions
        
        Sends to the movebase_client for sequential navigation
    """
    def follow_path_planner(self):
        #Read the point from the file sweeping_path.txt
        package_name = 'my_husky_package'
        file_name = '/include/sweeping_path.txt'
        path_from_file = self.get_data_from_file(package_name, file_name)

        move_base_goal_list = list()

        #Creates the MoveBaseGoal template
        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = 'utm'
        #Initializes the orientation. This needs to be changed
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1

        for i in range(len(path_from_file)):
            # Copies the object, so we can use the same template
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = path_from_file[i][0]
            move_base_goal_temp.target_pose.pose.position.y = path_from_file[i][1]

            rospy.loginfo(f"Next Point: ")
            rospy.loginfo(f"X: {path_from_file[i][0]}")
            rospy.loginfo(f"Y: {path_from_file[i][1]}")

            #Appends the list of MoveBaseGoals
            move_base_goal_list.append(move_base_goal_temp)

        rospy.loginfo("Starting to move")
        #Sending the MOveBaseGoals to the movebase_client for sequential navigation
        self.movebase_client(move_base_goal_list)


    """
        Reading the data from the file located inside a ROS package
        Input:
            package_name - The name of the ROS package
            file_name - The path to the file, including the name
        Output: The data points in the file in a list of tuples
    """
    def get_data_from_file(self, package_name: str, file_name: str):

        #Check if the package can be found, then get the path to the package
        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
        except rospkg.common.ResourceNotFound as e:
            rospy.logerr(f"Package not found: {e}")
            return None

        #Complete the path to the file
        file_path = package_path + file_name

        #Create the list to contain all data points
        gps_point_list = []

        #Try to open file
        try:
            with open(file_path, 'r') as file:
                #Go through every line in the file
                for line in file:
                    #Split the line with a comma
                    values = line.strip().split(',')
                    #Convert to float and save in a tuple
                    gps_point_tuple = (float(values[0]), float(values[1]))
                    #Append to a list that contains the data points
                    gps_point_list.append(gps_point_tuple)
        except FileNotFoundError as e:
            rospy.logerr(f"File not found: {e}")
            return None

        return gps_point_list

    """"
        Saves cartesian points to a file
        
        Converts the GPS points to cartesian
        
        Saves them to a file
    """
    def save_data_to_file(self):
        #Declare the service client
        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)
        self.pub_bug_talker.publish("Saving data to file")
        #Saves the cartesian coordinates to a file.
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'cartesian_cordinates.txt'), 'w')as file:
            for i in range(len(self.gps_points_list)):
                lat = self.gps_points_list[i][0]
                lon = self.gps_points_list[i][1]

                temp = utm.from_latlon(lat, lon)
                #respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
                file.write(f"{temp[0]} , {temp[1]}\n")
        self.pub_bug_talker.publish("Data saved to file")



    """
        Move base client for sequential navigation.
        
        Takes a list of MoveBaseAction goals as input.
        
        Creates a action client to the move_base action server supplied by the navigation stack
        
        Publishes a point
        Waits for a responce
        Publishes next point
        
    """
    def movebase_client(self, path: list):


        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        #Publishes the number of entries in the path to the debugger.
        self.pub_bug_talker.publish(f"The movebase list has {len(path)} entries")

        # Sends the goals to the action server.
        for goal in path:
            #Publishing the goal to the debugger
            self.pub_bug_talker.publish("Publishing goal")
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.x))
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.y))

            #Send the current goal to the action server
            client.send_goal(goal)

            # Wait for the move_base to respond with completed goal
            client.wait_for_result()

            # Wait one second after finishing goal. This is for testing purposes
            time.sleep(1)

            rospy.loginfo("Point Reached")

        self.pub_bug_talker.publish("Movement finished")





if __name__ == '__main__':
    rospy.init_node('follow_path')

    server = ActionServer()
    rospy.spin()

