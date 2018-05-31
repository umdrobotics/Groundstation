#!/usr/bin/env python

## How to install dependance
# pip2 install numpy/scipy/control/slycot
# if pip not working, use spt-get install python-scipy
# do not use python3, since ros do not support python3(needs a lot additional dependance)

import time
import numpy as np
import math

import scipy.linalg
import control

import rospy
from geometry_msgs.msg import PointStamped, Point, PoseWithCovariance
from truck_gps.msg import Array, Velocity, Acceleration, AttitudeQuaternion


dt = 0.1
q = 0.8

m = 3.1
g = 9.80665
kd = 0.0705

# lqrPub = rospy.Publisher("/LQR_K", Array, queue_size=10)
lqrPub = rospy.Publisher("/MPC_K", PoseWithCovariance, queue_size=10)


def publish_gain(K):
    # K = K.tolist()
    
    # msgMPC = Array()
    # msgMPC.header.stamp = rospy.Time.now()
    # msgMPC.data1 = K[0]
    # msgMPC.data2 = K[1]

    K = K.reshape(1,-1)[0]
    Kpad = np.zeros(36)
    Kpad[0:len(K)] = Kpad[0:len(K)] + K
    # print(Kpad)

    msgMPC = PoseWithCovariance()
    msgMPC.covariance = Kpad
    rospy.loginfo(msgMPC)
    lqrPub.publish(msgMPC)   



def timer_callback(event,K):

    publish_gain(K)



def main():

    rospy.init_node('MPC_controller', anonymous=True)


    A = np.array([[0, 0, 1, 0], \
                  [0, 0, 0, 1], \
                  [0, 0, -kd/m, 0 ], \
                  [0, 0, 0, -kd/m ]])
   
    B = np.array([[0, 0], \
                  [0, 0], \
                  [-g, 0], \
                  [0, -g]])

    Q = q * np.array([[1, 0, 0, 0], \
                      [0, 1, 0, 0], \
                      [0, 0, 8, 0], \
                      [0, 0, 0, 8]])

    R = (1-q) * np.array([[1, 0],\
                          [0, 1]])

    P = 20
    M = 3

    (nx,nu) = np.shape(B)
    mpc1 = MPCControllerBasic(A,B,Q,R,M,P)
    mpc1.Initialize(mpc1)

    stateError = np.zeros((nx*P),1);
    [~, ~, ~, K] = mpc1.ComputeOptimalInput(mpc1,stateError)
    

    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), timer_callback)

    rospy.spin()



if __name__ == '__main__':
    main()



    # #~ A = np.array([[1, 0, dt, 0], \
    #               #~ [0, 1, 0, dt], \
    #               #~ [0, 0, 1, 0 ], \
    #               #~ [0, 0, 0, 1 ]])
    # #~ 
    # #~ B = np.array([[dt**2/2, 0], \
    #               #~ [0, dt**2/2], \
    #               #~ [dt, 0], \
    #               #~ [0, dt]])
    # # A = np.array([[0, 0, 1, 0], \
    # #               [0, 0, 0, 1], \
    # #               [0, 0, 0, 0 ], \
    # #               [0, 0, 0, 0 ]])

    # # B = np.array([[0, 0], \
    # #               [0, 0], \
    # #               [1, 0], \
    # #               [0, 1]])

    # A = np.array([[0, 0 ], \
    #               [0, 0 ]])

    # B = np.array([[1, 0], \
    #               [0, 1]])

    
    # Q = q * np.array([[2, 0], \
    #                   [0, 2]])

    # R = (1-q) * np.array([[1, 0],\
    #                       [0, 1]])

    # #~ K, S, E = dlqr(A, B, Q, R)   #  discrete time lqr




    # def dlqr(self, A, B, Q, R):
    # """Solve the discrete time lqr controller.
 
 
    # x[k+1] = A x[k] + B u[k]
     
    # cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # """
    # #ref Bertsekas, p.151
 
    # #first, try to solve the ricatti equation
    # X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
     
    # #compute the LQR gain
    # K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
     
    # eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    # return K, X, eigVals
