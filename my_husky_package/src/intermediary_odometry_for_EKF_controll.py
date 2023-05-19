#!/usr/bin/env python3
import rospy
import rospkg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16

ekf_odometry = Odometry()   #Odometry gained from the EKF filter
gps_fix_status = None
husky_odometry = Odometry() #Current odometry from the husky
husky_odometry_minus_1 = Odometry() #Last odometry from the husky
calculated_odometry = Odometry()    #The difference between the husky odoms added to the EKF odom


def odom_relay(data):
    global gps_fix_status
    global ekf_odometry
    global husky_odometry
    global husky_odometry_minus_1
    global calculated_odometry

    husky_odometry_minus_1 = husky_odometry
    husky_odometry = data

    #rospy.loginfo(husky_odometry_minus_1.pose.pose.position.x)

    #If everything works as expected, send the odometry from the husky_velocity_controller
    #Else, send the latest EKF info to the EKF filter with the differance between the second to last and latest odometry
    if(gps_fix_status == 2):
        odom_pub.publish(data) #Just sends the odometry message that we recieved
    else:   #If there is no groung based augmentation. Send the last EKF message with the odometry difference
        #Find the difference between the incoming odometry, and the last
        x_difference = husky_odometry.pose.pose.position.x - husky_odometry_minus_1.pose.pose.position.x
        y_difference = husky_odometry.pose.pose.position.y - husky_odometry_minus_1.pose.pose.position.y

        #rospy.loginfo(f"X_difference: {x_difference}")
        #rospy.loginfo(f"Y_difference: {y_difference}")


        calculated_odometry = ekf_odometry
        #Add the difference
        calculated_odometry.pose.pose.position.x = ekf_odometry.pose.pose.position.x + x_difference
        calculated_odometry.pose.pose.position.y = ekf_odometry.pose.pose.position.y + y_difference

        #rospy.loginfo(f"Calculated Odometry X: {calculated_odometry.pose.pose.position.x}")


        #Publish it to the EKF.
        odom_pub.publish(calculated_odometry)


def odom_from_EKF(data):
    #Saves the odometry from the EKF in a global variable used elsewhere
    global ekf_odometry
    ekf_odometry = data

def odom_from_gps_relay(data):
    #Checks if there is a GPS signal whith ground augmentation. If not, it sends the last odometry.
    #This might not be needed. Testing required
    global gps_fix_status
    global ekf_odometry
    #Passes the odometry from the GPS if there is a RTK signal active.
    #Else it passes the last EKF signal.
    if(gps_fix_status == 2):
        gps_pub.publish(data)
    else:
        gps_pub.publish(calculated_odometry)

def gps_fix(data):
    #Checks the status of the GPS sender. We are looking for Status 2, which means it has ground based augmentation.
    global gps_fix_status
    gps_fix_status = data.status.status
    #fix_pub.publish(gps_fix_status) ##This is for testing purposes





if __name__ == '__main__':
    rospy.init_node('intermediary_odometry_for_EKF_controll')

    #Initializes all the subscribers
    odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_relay)
    ekf_sub = rospy.Subscriber('/global_ekf/odometry/filtered', Odometry, odom_from_EKF)
    gps_sub = rospy.Subscriber('/odometry/gps', Odometry, odom_from_gps_relay)
    fix_sub = rospy.Subscriber('/emlid/fix', NavSatFix, gps_fix)

    #Initializes all the publishers
    odom_pub = rospy.Publisher('/husky_odom/filtered', Odometry, queue_size=10)
    gps_pub = rospy.Publisher('/odometry/gps_filtered', Odometry, queue_size=10)
    #fix_pub = rospy.Publisher('/fix_status', Int16, queue_size=10) ##This is for testing purposes

    rospy.loginfo("Intermediary node for EKF filter is ready")

    rospy.spin()