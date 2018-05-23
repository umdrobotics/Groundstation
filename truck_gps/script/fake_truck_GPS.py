#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped



# GPS_lat = []
# GPS_lon = []
truckGpsPub = rospy.Publisher("/truck/location_GPS", PointStamped, queue_size=10)

# with open("/home/ubuntu/catkin_ws/src/truck_gps/script/lat.txt", "r") as f_lat:
#     for line in f_lat: 
#         inner_list = [float(elt.strip()) for elt in line.split(',')]
#         GPS_lat.append(inner_list)

# with open("/home/ubuntu/catkin_ws/src/truck_gps/script/lon.txt", "r") as f_lon:
#     for line in f_lon: 
#         inner_list = [float(elt.strip()) for elt in line.split(',')]
#         GPS_lon.append(inner_list)

# length = len(GPS_lat)
# i = 0
msgTruckLocation = PointStamped()

def TimerCallback(event):
    # global i 
    # if i > length-1:
    #     print("GPS_data publish end!!!!")
    # else:
    msgTruckLocation.header.frame_id = str("fake gps data")
    msgTruckLocation.header.stamp = rospy.Time.now()
    msgTruckLocation.point.x = 42.31458643
    msgTruckLocation.point.y = -83.22989117
    msgTruckLocation.point.z = 0
    truckGpsPub.publish(msgTruckLocation)

    # i = i + 1
    print "Msg:",msgTruckLocation
        # print "*********", i

def main():

    rospy.init_node('truck', anonymous=True)
    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), TimerCallback)
    rospy.spin()

if __name__ == '__main__':
    
    main()
