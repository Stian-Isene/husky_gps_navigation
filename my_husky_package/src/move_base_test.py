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
        self.pub_bug_talker = rospy.Publisher('bug_reporter', String, queue_size=10)

        #Starts the action server
        self.server.start()
        rospy.loginfo("Server is up and running")


    def start_action(self, goal: MovePathActionGoal):
        #This function starts when the action server is being called
        #In the goal there is an int, this will be used for controll.

        # Goal = 1: Move to the GPS coordinates found in the GPS file
        # Goal = 2: Move the path found in the pathplanner file
        package_name = 'gps_user_service'
        file_name = '/gps_coordinates.txt'
        self.gps_points_list = self.get_data_from_file(package_name, file_name)
        self.pub_bug_talker.publish(f"The list of gps points contains {len(self.gps_points_list)} entries")
        if goal.input == 1:
            self.move_to_gps_points()
        elif goal.input == 2:
            self.follow_path_planner()
        elif goal.input == 3:
            self.save_data_to_file()
        elif goal.input == 4:
            self.move_to_gps_points_using_cartesian()
        else:
            rospy.loginfo("Action not found")


        self.server.set_succeeded(self._result) #Sets the result of the action.

    def move_to_gps_points(self):

        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)

        move_base_goal_list = list()

        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = "utm"
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1




        cartesian_coordinates_list = list()
        for i in range(len(self.gps_points_list)):
            #rospy.loginfo(f"Latitude: {self.gps_points_list[i][0]}")
            #rospy.loginfo(f"Longitude: {self.gps_points_list[i][1]}")
            #respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
            #cartesian_coordinates.wait_for_service()
            #rospy.loginfo(f"x = {respond.x_axis}")
            #rospy.loginfo(f"y = {respond.y_axis}")

            # Copies the object, so we can use the same template
            #move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            #move_base_goal_temp.target_pose.pose.position.x = respond.x_axis
            #move_base_goal_temp.target_pose.pose.position.x = respond.y_axis
            #cartesian_coordinates_list.append((respond.x_axis, respond.y_axis))
            #move_base_goal_list.append(move_base_goal_temp)

            lat = self.gps_points_list[i][0]
            lon = self.gps_points_list[i][1]

            temp = utm.from_latlon(lat,lon)

            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = temp[0]
            move_base_goal_temp.target_pose.pose.position.y = temp[1]

            move_base_goal_list.append(move_base_goal_temp)


        rospy.loginfo("Starting to move")
        self.movebase_client(move_base_goal_list)


    def move_to_gps_points_using_cartesian(self):
        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)

        move_base_goal_list = list()

        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = "map"
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1




        cartesian_coordinates_list = list()
        for i in range(len(self.gps_points_list)):
            rospy.loginfo(f"Latitude: {self.gps_points_list[i][0]}")
            rospy.loginfo(f"Longitude: {self.gps_points_list[i][1]}")
            respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
            cartesian_coordinates.wait_for_service()
            rospy.loginfo(f"x = {respond.x_axis}")
            rospy.loginfo(f"y = {respond.y_axis}")

            # Copies the object, so we can use the same template
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = respond.x_axis
            move_base_goal_temp.target_pose.pose.position.x = respond.y_axis
            cartesian_coordinates_list.append((respond.x_axis, respond.y_axis))
            move_base_goal_list.append(move_base_goal_temp)



        rospy.loginfo("Starting to move")
        self.movebase_client(move_base_goal_list)


    def follow_path_planner(self):
        package_name = 'my_husky_package'
        file_name = '/include/sweeping_path.txt'
        path_from_file = self.get_data_from_file(package_name, file_name)

        move_base_goal_list = list()

        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = 'utm'
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

            move_base_goal_list.append(move_base_goal_temp)

        rospy.loginfo("Starting to move")
        self.movebase_client(move_base_goal_list)



    def get_data_from_file(self, package_name: str, file_name: str):
        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
        except rospkg.common.ResourceNotFound as e:
            rospy.logerr(f"Package not found: {e}")
            return None

        file_path = package_path + file_name

        gps_point_list = []

        try:
            with open(file_path, 'r') as file:
                for line in file:
                    values = line.strip().split(',')
                    gps_point_tuple = (float(values[0]), float(values[1]))
                    gps_point_list.append(gps_point_tuple)
        except FileNotFoundError as e:
            rospy.logerr(f"File not found: {e}")
            return None

        return gps_point_list


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




    def movebase_client(self, path: list):
        #Sends a list of MoveBaseGoals that we got from the pathplanner.



        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        self.pub_bug_talker.publish(f"The movebase list has {len(path)} entries")
        #client.send_goal(path[0])
        # Sends the goal to the action server.
        #publisher_hack = rospy.Publisher("/move_base/goal", MoveBaseGoal, queue=10)
        #publisher_hack.publish(path[0])
        for goal in path:
            self.pub_bug_talker.publish("Publishing goal")
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.x))
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.y))
            client.send_goal(goal)
            rospy.loginfo(f"Moving to position:")
            rospy.loginfo(f"X: {goal.target_pose.pose.position.x}")
            rospy.loginfo(f"Y: {goal.target_pose.pose.position.y}")


            #rospy.loginfo(goal.target_pose.pose.position.x)
            wait = client.wait_for_result()
            client.wait_for_result()
            time.sleep(1)
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                rospy.loginfo(client.get_result())
            rospy.loginfo("Point Reached")

        self.pub_bug_talker.publish("Movement finished")


        # #client.send_goal(goal)
        # # Waits for the server to finish performing the action.
        # wait = client.wait_for_result()
        # # If the result doesn't arrive, assume the Server is not available
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     # Result of executing the action
        #     return client.get_result()

        # If the python node is executed as main process (sourced directly)


if __name__ == '__main__':
    rospy.init_node('follow_path')

    server = ActionServer()
    rospy.spin()

    # try:
    #     # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    #     rospy.init_node('movebase_client_py')
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Goal execution done!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")