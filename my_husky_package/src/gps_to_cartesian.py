#!/usr/bin/env python3
import rospy
from my_husky_messages.srv import GPSToCartesian, GPSToCartesianRequest, GPSToCartesianResponse
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class GPSToActionServer:



    def __init__(self):
        #Initializes the publisher and subscriber that is used to get the messages.
        self.pub = rospy.Publisher('gps_to_cartesian/input', NavSatFix, queue_size=10)
        self.sub = rospy.Subscriber('gps_to_cartesian/output', Odometry, callback=self.callback_from_navsat)

        #Initializes the service
        self.s = rospy.Service('gps_to_cartesian_service', GPSToCartesian, self.callback_gps_to_cartesian)

        #Initialies the variables used.
        self.wait_for_response = bool
        self.response_x_axis = float()
        self.response_y_axis = float()
        rospy.loginfo("Server up and running")


    def callback_gps_to_cartesian(self, req):
        #This is run when the service is being called
        #The request is a message with latitude and longitude

        #Converts the request into a NavSatFix message object and addends a frame_id
        self.coordinates = NavSatFix()
        self.coordinates.latitude = req.latitude
        self.coordinates.longitude = req.longitude
        self.coordinates.header.frame_id = "reach_rs"
        self.coordinates.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
        self.coordinates.status.service = 1

        #Publishes to the navsat_transform node set up to handle this transform
        self.pub.publish(self.coordinates)

        #Logic for timing the response
        self.wait_for_response = True
        self.timer_past = rospy.get_time()

        #Wait for a predetermined amount, here 10 seconds
        while(self.wait_for_response):
            self.timer_now = rospy.get_time()
            if(self.timer_now - self.timer_past > 10):
                self.wait_for_response = False

        #Return the response from the navsat_transform node
        return GPSToCartesianResponse(x_axis=self.response_x_axis, y_axis=self.response_y_axis)

    def callback_from_navsat(self, req):
        #Listener to get the response from the navsat_transform node

        #Extrapolate the cartesian coordinates
        self.response_x_axis = req.pose.pose.position.x
        self.response_y_axis = req.pose.pose.position.y
        rospy.loginfo("Coordinates arrived")
        self.wait_for_response = False









if __name__ == '__main__':
    rospy.init_node("gps_to_cartesian_service_node")
    server = GPSToActionServer()
    rospy.spin()
