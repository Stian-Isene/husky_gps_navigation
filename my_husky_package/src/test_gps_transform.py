#!/usr/bin/env python3
import copy

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from my_husky_messages.msg import MovePathActionGoal, MovePathActionResult, MovePathActionFeedback, MovePathAction
from my_husky_messages.srv import GPSToCartesian, GPSToCartesianRequest, GPSToCartesianResponse


def test():
    rospy.wait_for_service('gps_to_cartesian_service')
    cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)
    
    
    respond = cartesian_coordinates(68.0, 9.0)
    rospy.loginfo(f"x = {respond.x_axis}")
    rospy.loginfo(f"y = {respond.y_axis}")

    respond = cartesian_coordinates(68.5, 9.5)
    rospy.loginfo(f"x = {respond.x_axis}")
    rospy.loginfo(f"y = {respond.y_axis}")

if __name__ == '__main__':
    rospy.init_node("Testing123")
    test()
    rospy.spin()