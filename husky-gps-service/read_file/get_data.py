#! /usr/bin/env python3
import rospy
import rospkg

def get_data_from_file():
    try:
        rp = rospkg.RosPack()
        package_path = rp.get_path('gps_user_service')
    except rospkg.common.ResourceNotFound as e:
        rospy.logerr(f"Package not found: {e}")
        return None

    file_path = package_path + '/gps_coordinates.txt'

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

if __name__ == '__main__':
    ls = get_data_from_file()
    if ls is not None:
        print(ls)
    else:
        rospy.logerr("No data was retrieved from the file.")

