#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import NavSatFix
from gps_user_service.srv import SaveGPS

dir_path = os.path.dirname(os.path.realpath(__file__))

def save_gps(req):
    # Open a file for writing
    #f = open("gps_coordinates.txt", "w")
    f = open(os.path.join(dir_path, 'gps_coordinates.txt'), 'w')
    # Loop through the list of GPS coordinates in the request
    for gps in req.gps_coordinates:
        # Write the GPS coordinates to the file
        f.write("{0},{1}\n".format(gps.latitude, gps.longitude))
    # Close the file
    f.close()
    # Return a success message
    return {"success": True}


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("gps_user_service_server")
    # Create the service
    rospy.Service("save_gps", SaveGPS, save_gps)
    # Spin the node
    rospy.spin()