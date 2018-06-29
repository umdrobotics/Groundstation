#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
# from apriltags2_ros.msg import AprilTagDetectionArray

import socket
import re

TCP_IP = '127.0.0.1'
TCP_PORT = 9999
BUFFER_SIZE = 1024


# Setup ROS
rospy.init_node('Apriltag_server', anonymous=True)
pub1 = rospy.Publisher("/usb_cam/tag_detections_socket", PoseStamped, 10)

print "Initialized ROS, try to connect......"

# Setup Socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
print "Connected to client:", addr, "! Starting sending data."

vecTagDetections = AprilTagDetectionArray()


while 1:
    data = s.recv(BUFFER_SIZE)

    if not data: break

    splitData = PoseStamped()

    # Try to split data
    if "###" in data and "!!!" in data:
        
        indicesMsgHead = [m.start() for m in re.finditer("###", data)]
        indicesMsgEnd = [m.start() for m in re.finditer("!!!", data)]

        firstMsg = data[indicesMsgHead[0]+3 : indicesMsgEnd[0]]
        indicesComma = [m.start() for m in re.finditer(",", data)]

        if len(indicesComma) != 6:
            raiseError("Msg inside is wrong")

        vecTagDetections.pose.position.x    = double(data[0 : indicesComma[0]])
        vecTagDetections.pose.position.y    = double(data[indicesComma[0] : indicesComma[1]])
        vecTagDetections.pose.position.z    = double(data[indicesComma[1] : indicesComma[2]])
        vecTagDetections.pose.orientation.x = double(data[indicesComma[2] : indicesComma[3]])
        vecTagDetections.pose.orientation.y = double(data[indicesComma[3] : indicesComma[4]])
        vecTagDetections.pose.orientation.z = double(data[indicesComma[4] : indicesComma[5]])
        vecTagDetections.pose.orientation.w = double(data[indicesComma[5] : -1])
        pub1.publish(vecTagDetections)
        rospy.loginfo(vecTagDetections)



rospy.spin()
conn.close()


