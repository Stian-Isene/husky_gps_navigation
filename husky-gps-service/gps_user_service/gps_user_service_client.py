#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from gps_user_service.srv import SaveGPS

def gps_user_service_client():
    # Initialize the node
    rospy.init_node('gps_user_service_client')

    # Wait for the service to become available
    rospy.wait_for_service('save_gps')

    # Create the service proxy
    save_gps = rospy.ServiceProxy('save_gps', SaveGPS)

    # Create sample GPS coordinates
    gps_points = [
        NavSatFix(latitude=59.9127, longitude=10.7461),
        #NavSatFix(latitude=60.3913, longitude=5.3221),
        #NavSatFix(latitude=58.9699, longitude=5.7331),
        #NavSatFix(latitude=63.4305, longitude=10.3951)
    ]

    try:
        # Call the service with the sample GPS coordinates
        response = save_gps(gps_points)

        if response.success:
            print("GPS coordinates saved successfully")
        else:
            print("Failed to save GPS coordinates")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    gps_user_service_client()
