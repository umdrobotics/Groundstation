#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from apriltags2_ros.msg import AprilTagDetectionArray

import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 9999
BUFFER_SIZE = 1024

# Core function: everytime received a msg, send it out in socket
def callback(vecTagDetections):

    if not vecTagDetections: # if vector is empty
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

    conn.send(data)



# Setup ROS
rospy.init_node('Apriltag_server', anonymous=True)
print "Initialized ROS, waiting for connections......"


# Setup Socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
err_log = s.bind((TCP_IP, TCP_PORT))
if err_log != 0:
    print "Error in binding."
    conn.close()
    
err_log = s.listen(5) # non-blocking, just tell the kernel how many clients to wait
if err_log != 0:
    print "Error in listening."
    conn.close()
       
conn, addr = s.accept() # blocking, until this func get one conn & addr from the establised connections queue
print "Connected to client:", addr, "! Start sending data."



sub1 = rospy.Subscriber("/usb_cam/tag_detections", AprilTagDetectionArray, callback)

rospy.spin()
conn.close()


