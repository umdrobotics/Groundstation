#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from apriltags2_ros.msg import AprilTagDetectionArray

import socket

import time



TCP_IP = '192.168.1.17'
TCP_PORT = 9999
BUFFER_SIZE = 1024

counter = 0


# Core function: everytime received a msg, send it out in socket
def callback(vecTagDetections):

    global counter

    if not vecTagDetections.detections: # if vector is empty
        return

    # Encode data and publish
    data = "###"
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.position.x) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.position.y) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.position.z) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.orientation.x) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.orientation.y) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.orientation.z) + ","
    data = data + str(vecTagDetections.detections[0].pose.pose.pose.orientation.w)
    data = data + "!!!"

    try:
        conn.send(data)
    finally:
        counter += 1
        print(time.time(), data, counter)
    




# Setup ROS
rospy.init_node('Apriltag_server', anonymous=True)
print("Initialized ROS, waiting for connections......")


# Setup Socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    s.bind((TCP_IP, TCP_PORT))
    s.listen(5) # non-blocking, just tell the kernel how many clients to wait
except socket.error as e:
    raise ValueError(e)  


conn, addr = s.accept() # blocking, until this func get one conn & addr from the establised connections queue
print("Connected to client:", addr, "Start sending data.")



sub1 = rospy.Subscriber("/usb_cam/tag_detections", AprilTagDetectionArray, callback)

rospy.spin()



