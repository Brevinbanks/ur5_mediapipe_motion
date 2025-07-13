#!/usr/bin/env python
# Main script for the cv_ur5_project package
# This file contains the main class for the UR5 robot motion control
# It first moves the ur5 to the initial joint state, then executes the IK solver
# The IK solver uses the forward kinematics and Jacobian to compute the joint angles
# The IK solver then uses the inverse kinematics to compute the end effector pose
# Goal poses for the IK solver are regularly updated based on the transform callback
# recieved from the /mediapipe_transform topic. If it detects frames published by the topic
# post the initialization phase, it will publish goal poses to the joint states
# Gripper control is also implemented in this file. The gripper is controlled by the
# /gripper_condition topic, which is then converted to a std_msgs message and published
# to the /gripper/joint_states topic.

# MAKE SURE ur5_controller.py IS RUNNING BEFORE RUNNING THIS SCRIPT

import rospy
from std_msgs.msg import Float64MultiArray, Int32, Float64
from sensor_msgs.msg import JointState
import tf2_ros
import tf.transformations as tf
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
from urdf_parser_py.urdf import Robot
from ur5_urdf_parser import ur5_urdf_parser
import ur5_FKandJacob as ur5FK
import time

class Iter_IK_ur5():

    def __init__(self):
        self.joint_positions = [] # ur5 joint positions
        self.current_pose = np.eye(4) # current end effector pose
        self.prev_pose = np.eye(4) # previous end effector pose
        self.J = np.eye(6) # Jacobian matrix
        self.dq = np.array([[0],[1],[0],[0],[0],[0]]) # joint velocity
        self.q = np.array([[0],[0],[0],[0],[0],[0]]) # joint angles
        self.goal_pose = np.eye(4) # goal pose for IK solver
        self.ready_for_mediapipe_input = False # flag to indicate if goal poses are ready once home pose is achieved
        self.gripper_state = 0 # gripper state

    def ForwardKinematics(self, q1, q2, q3, q4, q5, q6):
        # Compute the forward kinematics for the UR5 robot
        E = np.zeros((4,4))
        E[0][0] = -np.sin(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))-np.sin(q2+q3+q4)*np.cos(q1)*np.cos(q6)
        E[0][1] = -np.cos(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))+np.sin(q2+q3+q4)*np.cos(q1)*np.sin(q6)
        E[0][2] = -np.cos(q5)*np.sin(q1)+np.cos(q2+q3+q4)*np.cos(q1)*np.sin(q5)
        E[0][3] = np.sin(q1)*(-1.0915E-1)+np.cos(q1)*np.cos(q2)*(1.7E+1/4.0E+1)-np.cos(q5)*np.sin(q1)*2.21617731387E-1-np.cos(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))*1.80755331E-4-np.sin(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))*1.26590643E-3+np.cos(q2+q3+q4)*np.cos(q1)*np.sin(q5)*2.21617731387E-1-np.sin(q2+q3+q4)*np.cos(q1)*np.cos(q6)*1.26590643E-3+np.sin(q2+q3+q4)*np.cos(q1)*np.sin(q6)*1.80755331E-4-np.cos(q2+q3)*np.cos(q1)*np.sin(q4)*9.465E-2-np.sin(q2+q3)*np.cos(q1)*np.cos(q4)*9.465E-2+np.cos(q1)*np.cos(q2)*np.cos(q3)*3.9225E-1-np.cos(q1)*np.sin(q2)*np.sin(q3)*3.9225E-1
        E[1][0] = np.sin(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))-np.sin(q2+q3+q4)*np.cos(q6)*np.sin(q1)
        E[1][1] = np.cos(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))+np.sin(q2+q3+q4)*np.sin(q1)*np.sin(q6)
        E[1][2] = np.cos(q1)*np.cos(q5)+np.cos(q2+q3+q4)*np.sin(q1)*np.sin(q5)
        E[1][3] = np.cos(q1)*1.0915E-1+np.cos(q1)*np.cos(q5)*2.21617731387E-1+np.cos(q2)*np.sin(q1)*(1.7E+1/4.0E+1)+np.cos(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))*1.80755331E-4+np.sin(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))*1.26590643E-3-np.sin(q1)*np.sin(q2)*np.sin(q3)*3.9225E-1+np.cos(q2+q3+q4)*np.sin(q1)*np.sin(q5)*2.21617731387E-1-np.sin(q2+q3+q4)*np.cos(q6)*np.sin(q1)*1.26590643E-3+np.sin(q2+q3+q4)*np.sin(q1)*np.sin(q6)*1.80755331E-4-np.cos(q2+q3)*np.sin(q1)*np.sin(q4)*9.465E-2-np.sin(q2+q3)*np.cos(q4)*np.sin(q1)*9.465E-2+np.cos(q2)*np.cos(q3)*np.sin(q1)*3.9225E-1
        E[2][0] = -np.cos(q2+q3+q4)*np.cos(q6)+np.sin(q2+q3+q4)*np.cos(q5)*np.sin(q6)
        E[2][1] = np.cos(q2+q3+q4)*np.sin(q6)+np.sin(q2+q3+q4)*np.cos(q5)*np.cos(q6)
        E[2][2] = -np.sin(q2+q3+q4)*np.sin(q5)
        E[2][3] = np.sin(q2+q3)*(-3.9225E-1)-np.sin(q2)*(1.7E+1/4.0E+1)+np.sin(q2+q3+q4)*np.cos(q5+q6)*9.03776655E-5+np.sin(q2+q3+q4)*np.sin(q5+q6)*6.32953215E-4-np.cos(q2+q3+q4)*np.cos(q6)*1.26590643E-3+np.cos(q2+q3+q4)*np.sin(q6)*1.80755331E-4-np.sin(q2+q3+q4)*np.sin(q5)*2.21617731387E-1+np.cos(q5-q6)*np.sin(q2+q3+q4)*9.03776655E-5-np.sin(q5-q6)*np.sin(q2+q3+q4)*6.32953215E-4-np.cos(q2+q3)*np.cos(q4)*9.465E-2+np.sin(q2+q3)*np.sin(q4)*9.465E-2+8.9159E-2
        E[3][3] = 1.0

        E = np.round(E, decimals=4)

        return E

    def ForwardKinematicsInverse(self, q1, q2, q3, q4, q5, q6):
        # Compute the inverse of the forward kinematics for the UR5 robot
        E = np.zeros((4,4))
        E[0][0] = -np.sin(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))-np.sin(q2+q3+q4)*np.cos(q1)*np.cos(q6)
        E[0][1] = np.sin(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))-np.sin(q2+q3+q4)*np.cos(q6)*np.sin(q1)
        E[0][2] = -np.cos(q2+q3+q4)*np.cos(q6)+np.sin(q2+q3+q4)*np.cos(q5)*np.sin(q6)
        E[0][3] = np.cos(q6)*(-9.465E-2)+np.cos(q6)*np.sin(q4)*3.9225E-1-np.sin(q5)*np.sin(q6)*1.0915E-1+np.cos(q3)*np.cos(q6)*np.sin(q4)*(1.7E+1/4.0E+1)+np.cos(q4)*np.cos(q6)*np.sin(q3)*(1.7E+1/4.0E+1)+np.cos(q4)*np.cos(q5)*np.sin(q6)*3.9225E-1+np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q6)*8.9159E-2+np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q6)*(1.7E+1/4.0E+1)-np.cos(q2)*np.cos(q6)*np.sin(q3)*np.sin(q4)*8.9159E-2-np.cos(q3)*np.cos(q6)*np.sin(q2)*np.sin(q4)*8.9159E-2-np.cos(q4)*np.cos(q6)*np.sin(q2)*np.sin(q3)*8.9159E-2-np.cos(q5)*np.sin(q3)*np.sin(q4)*np.sin(q6)*(1.7E+1/4.0E+1)-np.cos(q2)*np.cos(q3)*np.cos(q5)*np.sin(q4)*np.sin(q6)*8.9159E-2-np.cos(q2)*np.cos(q4)*np.cos(q5)*np.sin(q3)*np.sin(q6)*8.9159E-2-np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)*np.sin(q6)*8.9159E-2+np.cos(q5)*np.sin(q2)*np.sin(q3)*np.sin(q4)*np.sin(q6)*8.9159E-2-1.26590643E-3
        E[1][0] = -np.cos(q6)*(np.sin(q1)*np.sin(q5)+np.cos(q2+q3+q4)*np.cos(q1)*np.cos(q5))+np.sin(q2+q3+q4)*np.cos(q1)*np.sin(q6)
        E[1][1] = np.cos(q6)*(np.cos(q1)*np.sin(q5)-np.cos(q2+q3+q4)*np.cos(q5)*np.sin(q1))+np.sin(q2+q3+q4)*np.sin(q1)*np.sin(q6)
        E[1][2] = np.cos(q2+q3+q4)*np.sin(q6)+np.sin(q2+q3+q4)*np.cos(q5)*np.cos(q6)
        E[1][3] = np.sin(q6)*9.465E-2-np.cos(q6)*np.sin(q5)*1.0915E-1-np.sin(q4)*np.sin(q6)*3.9225E-1+np.cos(q4)*np.cos(q5)*np.cos(q6)*3.9225E-1-np.cos(q3)*np.sin(q4)*np.sin(q6)*(1.7E+1/4.0E+1)-np.cos(q4)*np.sin(q3)*np.sin(q6)*(1.7E+1/4.0E+1)+np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6)*(1.7E+1/4.0E+1)-np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q6)*8.9159E-2-np.cos(q5)*np.cos(q6)*np.sin(q3)*np.sin(q4)*(1.7E+1/4.0E+1)+np.cos(q2)*np.sin(q3)*np.sin(q4)*np.sin(q6)*8.9159E-2+np.cos(q3)*np.sin(q2)*np.sin(q4)*np.sin(q6)*8.9159E-2+np.cos(q4)*np.sin(q2)*np.sin(q3)*np.sin(q6)*8.9159E-2-np.cos(q2)*np.cos(q3)*np.cos(q5)*np.cos(q6)*np.sin(q4)*8.9159E-2-np.cos(q2)*np.cos(q4)*np.cos(q5)*np.cos(q6)*np.sin(q3)*8.9159E-2-np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6)*np.sin(q2)*8.9159E-2+np.cos(q5)*np.cos(q6)*np.sin(q2)*np.sin(q3)*np.sin(q4)*8.9159E-2-1.80755331E-4
        E[2][0] = -np.cos(q5)*np.sin(q1)+np.cos(q2+q3+q4)*np.cos(q1)*np.sin(q5)
        E[2][1] = np.cos(q1)*np.cos(q5)+np.cos(q2+q3+q4)*np.sin(q1)*np.sin(q5)
        E[2][2] = -np.sin(q2+q3+q4)*np.sin(q5)
        E[2][3] = np.cos(q5)*(-1.0915E-1)-np.cos(q4)*np.sin(q5)*3.9225E-1+np.sin(q3)*np.sin(q4)*np.sin(q5)*(1.7E+1/4.0E+1)-np.cos(q3)*np.cos(q4)*np.sin(q5)*(1.7E+1/4.0E+1)+np.cos(q2)*np.cos(q3)*np.sin(q4)*np.sin(q5)*8.9159E-2+np.cos(q2)*np.cos(q4)*np.sin(q3)*np.sin(q5)*8.9159E-2+np.cos(q3)*np.cos(q4)*np.sin(q2)*np.sin(q5)*8.9159E-2-np.sin(q2)*np.sin(q3)*np.sin(q4)*np.sin(q5)*8.9159E-2-2.21617731387E-1
        E[3][3] = 1.0
        

        return E

    def Jacobian(self, q1, q2, q3, q4, q5, q6):
        # Compute the Jacobian matrix for the UR5 robot
        J = np.zeros((6,6))
        J[0][1] = np.cos(q1) * (-8.9159E-2)
        J[0][2] = (np.cos(q1) * (np.sin(q2) * 1.531223873305969E+17 - 3.212291513413808E+16)) / 3.602879701896397E+17
        J[0][3] = (np.cos(q1) * (np.sin(q2 + q3) * 3.533073907672154E+18 + np.sin(q2) * 3.828059683264922E+18 - 8.030728783534521E+17)) / 9.007199254740992E+18
        J[0][4] = np.sin(q1 - q4) * (-1.96125E-1) - np.sin(q1 + q3 + q4) * (1.7E+1 / 8.0E+1) + np.sin(-q1 + q3 + q4) * (1.7E+1 / 8.0E+1) - np.cos(q1 + q2 + q3 + q4) * 9.91545E-2 - np.sin(q1 + q4) * 1.96125E-1 - np.cos(-q1 + q2 + q3 + q4) * 9.995499999999999E-3
        J[0][5] = np.cos(q1) * np.cos(q5) * (-8.9159E-2) + np.sin(q1) * np.sin(q5) * 9.465E-2 - np.sin(q1) * np.sin(q4) * np.sin(q5) * 3.9225E-1 + np.cos(q1) * np.cos(q5) * np.sin(q2) * (1.7E+1 / 4.0E+1) + np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) * 3.9225E-1 + np.cos(q1) * np.cos(q3) * np.cos(q5) * np.sin(q2) * 3.9225E-1 - np.cos(q3) * np.sin(q1) * np.sin(q4) * np.sin(q5) * (1.7E+1 / 4.0E+1) - np.cos(q4) * np.sin(q1) * np.sin(q3) * np.sin(q5) * (1.7E+1 / 4.0E+1) + np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5) * 9.465E-2 - np.cos(q1) * np.cos(q2) * np.cos(q3) * np.sin(q4) * np.sin(q5) * 1.0915E-1 - np.cos(q1) * np.cos(q2) * np.cos(q4) * np.sin(q3) * np.sin(q5) * 1.0915E-1 - np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) * np.sin(q4) * 9.465E-2 - np.cos(q1) * np.cos(q3) * np.cos(q4) * np.sin(q2) * np.sin(q5) * 1.0915E-1 - np.cos(q1) * np.cos(q3) * np.cos(q5) * np.sin(q2) * np.sin(q4) * 9.465E-2 - np.cos(q1) * np.cos(q4) * np.cos(q5) * np.sin(q2) * np.sin(q3) * 9.465E-2 - np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5) * 8.9159E-2 + np.cos(q1) * np.sin(q2) * np.sin(q3) * np.sin(q4) * np.sin(q5) * 1.0915E-1 + np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4) * np.sin(q5) * 8.9159E-2 + np.cos(q3) * np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5) * 8.9159E-2 + np.cos(q4) * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q5) * 8.9159E-2
        J[1][1] = np.sin(q1) * (-8.9159E-2)
        J[1][2] = (np.sin(q1) * (np.sin(q2) * 1.531223873305969E+17 - 3.212291513413808E+16)) / 3.602879701896397E+17
        J[1][3] = (np.sin(q1) * (np.sin(q2 + q3) * 3.533073907672154E+18 + np.sin(q2) * 3.828059683264922E+18 - 8.030728783534521E+17)) / 9.007199254740992E+18
        J[1][4] = np.cos(q1 - q4) * 1.96125E-1 + np.cos(q1 + q3 + q4) * (1.7E+1 / 8.0E+1) + np.cos(-q1 + q3 + q4) * (1.7E+1 / 8.0E+1) - np.sin(q1 + q2 + q3 + q4) * 9.91545E-2 + np.cos(q1 + q4) * 1.96125E-1 + np.sin(-q1 + q2 + q3 + q4) * 9.995499999999999E-3
        J[1][5] = np.cos(q1)*np.sin(q5)*(-9.465E-2)-np.cos(q5)*np.sin(q1)*8.9159E-2+np.cos(q5)*np.sin(q1)*np.sin(q2)*(1.7E+1/4.0E+1)+np.cos(q1)*np.sin(q4)*np.sin(q5)*3.9225E-1+np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3)*3.9225E-1+np.cos(q3)*np.cos(q5)*np.sin(q1)*np.sin(q2)*3.9225E-1+np.cos(q1)*np.cos(q3)*np.sin(q4)*np.sin(q5)*(1.7E+1/4.0E+1)+np.cos(q1)*np.cos(q4)*np.sin(q3)*np.sin(q5)*(1.7E+1/4.0E+1)+np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q5)*8.9159E-2+np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q1)*9.465E-2-np.cos(q1)*np.cos(q2)*np.sin(q3)*np.sin(q4)*np.sin(q5)*8.9159E-2-np.cos(q1)*np.cos(q3)*np.sin(q2)*np.sin(q4)*np.sin(q5)*8.9159E-2-np.cos(q1)*np.cos(q4)*np.sin(q2)*np.sin(q3)*np.sin(q5)*8.9159E-2-np.cos(q2)*np.cos(q3)*np.sin(q1)*np.sin(q4)*np.sin(q5)*1.0915E-1-np.cos(q2)*np.cos(q4)*np.sin(q1)*np.sin(q3)*np.sin(q5)*1.0915E-1-np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3)*np.sin(q4)*9.465E-2-np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q2)*np.sin(q5)*1.0915E-1-np.cos(q3)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q4)*9.465E-2-np.cos(q4)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q3)*9.465E-2+np.sin(q1)*np.sin(q2)*np.sin(q3)*np.sin(q4)*np.sin(q5)*1.0915E-1
        J[2][2] = np.cos(q2)*(1.7E+1/4.0E+1)
        J[2][3] = np.cos(q2+q3)*3.9225E-1+np.cos(q2)*(1.7E+1/4.0E+1)
        J[2][4] = np.sin(q2+q3+q4)*1.0915E-1
        J[2][5] = np.cos(q2-q5)*(1.7E+1/8.0E+1)+np.cos(q2+q3+q5)*1.96125E-1+np.cos(q2+q3-q5)*1.96125E-1-np.sin(q2+q3+q4+q5)*1.019E-1+np.cos(q2+q5)*(1.7E+1/8.0E+1)+np.sin(q2+q3+q4-q5)*7.25E-3
        J[3][1] = -np.sin(q1)
        J[3][2] = -np.sin(q1)
        J[3][3] = -np.sin(q1)
        J[3][4] = -np.sin(q2+q3+q4)*np.cos(q1)
        J[3][5] = -np.cos(q5)*np.sin(q1)+np.cos(q2+q3+q4)*np.cos(q1)*np.sin(q5)
        J[4][1] = np.cos(q1)
        J[4][2] = np.cos(q1)
        J[4][3] = np.cos(q1)
        J[4][4] = -np.sin(q2+q3+q4)*np.sin(q1)
        J[4][5] = np.cos(q1)*np.cos(q5)+np.cos(q2+q3+q4)*np.sin(q1)*np.sin(q5)
        J[5][0] = 1.0
        J[5][4] = -np.cos(q2+q3+q4)
        J[5][5] = -np.sin(q2+q3+q4)*np.sin(q5)

        return J



    # Implement the iterative IK solver
    def iterative_ik_solver(self, error):
        
        # Initialize joint angles (q) to some initial guess
        q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
                      
        # Set the max error per step and convergence threshold
        Dmax = .2
        p_error = error # previous error
        error = self.clamp_mag(error,Dmax) # clamp error magnitude 
        
        # Compute Jacobian matrix
        self.J = ur5FK.Jacobian(self.q)
  
        # Compute the error
        dx = self.compute_pose_error(self.prev_pose, self.current_pose)

        # Update the Jacobian matrix using Broyden's method (lambda_damp is a damping factor on a modified the damped-least squares method)
        lambda_damp = 0.01
        # Update joint angles using where K is a diagonal matrix with the damping factors
        K = np.diag([5,5,5,1,1,1])
        self.dq = np.dot(np.dot(np.linalg.inv(np.dot(self.J.T , self.J) + lambda_damp**2 * np.eye(6)), self.J.T),np.dot(K,error))
        newq = q
        newq += self.dq
        alpha = 0.9

        # Apply a low-pass filter to the joint angles to reduce jitter in the control loop. Helps with slower machines. Can remove on faster processors
        self.q =  alpha * newq + (1 - alpha) * self.q
            
        return self.q



    def clamp_mag(self, error,Dmax):
        # Clamp the error magnitude to Dmax
        if np.linalg.norm(error)<Dmax:
            error=error
        else:
            error = Dmax*error/np.linalg.norm(error)
        return error



    # Define compute_pose_error function
    def compute_pose_error(self, current_pose, goal_pose):

        rot_mat = current_pose[0:3,0:3]
        rot_vec = tf.euler_from_matrix(rot_mat)

        current_pose_vec = np.array([current_pose[0,3],current_pose[1,3],current_pose[2,3],rot_vec[0],rot_vec[1],rot_vec[2]])

        goal_rot_mat = self.goal_pose[0:3,0:3]
        goal_rot_vec = tf.euler_from_matrix(goal_rot_mat)

        goal_pose_vec = np.array([self.goal_pose[0,3],self.goal_pose[1,3],self.goal_pose[2,3],goal_rot_vec[0],goal_rot_vec[1],goal_rot_vec[2]])

        position_error = goal_pose_vec[:3] - current_pose_vec[:3]
        orientation_error = goal_pose_vec[3:] - current_pose_vec[3:]  # Assuming euler angles for simplicity
        return np.concatenate((position_error, orientation_error))

    

    # Define interpolate_pose function
    def interpolate_pose(self, start_pose, goal_pose, alpha):
        interpolated_position = start_pose[:3] * (1 - alpha) + self.goal_pose[:3] * alpha
        interpolated_orientation = start_pose[3:] * (1 - alpha) + self.goal_pose[3:] * alpha
        return np.concatenate((interpolated_position, interpolated_orientation))



    def ur5_joint_sub_callback(self, msg):
        self.joint_positions = msg.position
   
    def transform_callback(self, msg):
        q = [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w
        ]

        # construct 4x4 transformation matrix from pose (not rotation comes in from quaternion so we need to convert)
        pose = np.eye(4)
        pose[0,3] = msg.transform.translation.x
        pose[1,3] = msg.transform.translation.y
        pose[2,3] = msg.transform.translation.z
        pose[0:3,0:3] = tf.quaternion_matrix(q)[0:3,0:3]
        if self.ready_for_mediapipe_input:
            self.goal_pose = pose

    def gripper_callback(self, msg):
        # Gripper callback function
        self.gripper_state = msg.data

    def publisher(self): # Main function to publish joint states
        # Subscribe to the /mediapipe_transform topic
        rospy.Subscriber('/mediapipe_transform', TransformStamped, self.transform_callback)
        # Subscribe to the /gripper_condition topic
        rospy.Subscriber('/gripper_condition', Int32, self.gripper_callback)
        # Initialize the joint state publisher
        rospy.init_node('joint_states_publisher', anonymous=True)
        pub = rospy.Publisher('cv_des_joint_states', Float64MultiArray, queue_size=10)
        # Initialize the subscriber for the /joint_states topic
        rospy.Subscriber('joint_states', JointState, self.ur5_joint_sub_callback)
        convergence_threshold = 0.001
        # Initialize the gripper publisher
        gripper_pub = rospy.Publisher('/Slider1_effort_controller/command', Float64, queue_size=10)
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        # Initialize the transformation that will be created to use in the IK solver
        transform = geometry_msgs.msg.TransformStamped()
        # define the transformation frame
        transform.header.frame_id = "base_link"  # Assuming models are defined in the base_link frame
        transform.child_frame_id = "ik_goal"
        # Parse the URDF file to understand the robot structure
        parser = ur5_urdf_parser()
        # Publish the joint states at 10 Hz. Can change if your machine is faster
        rate = rospy.Rate(10)  # 10Hz
        # Initialize the goal poses for the IK solver. Dummy values for now
        self.goal_pose = np.array([[-1.00000000e+00, -9.79311649e-12, -4.89652763e-12, 8.17250000e-01]
                              ,[-4.89652763e-12, 2.99829594e-28, 1.00000000e+00, 1.91450000e-01]
                              ,[-9.79311649e-12, 1.00000000e+00, -4.79525653e-23, -5.49100000e-03]
                              ,[0.0, 0.0, 0.0, 1.0]])
        
        # Wait a second for the robot to initialize and the simulation to settle
        rospy.sleep(1)
        # Create the joint state message to publish
        joint_states = Float64MultiArray()
        # These values are the homing/start position the robot will move to on initialization
        joint_states.data = [-1.91, -1.4, 2.15, -2.25, -1.57, -0.38]   # Modify with desired values
        # Publish the joint states
        pub.publish(joint_states)
        # Initialize the error variable, something gretter than 1 to prevent the loop from skipping
        error = 5
        print("putting robot in position")
        # Break out will help the robot to stop moving if it gets stuck homing forever
        break_out = False
        break_out_time = time.time()
        while np.linalg.norm(error)>1 and break_out == False:
            # Update the goal poses for each step toward the home position
            self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
            self.goal_pose = ur5FK.ForwardKinematics(joint_states.data)
            self.current_pose = ur5FK.ForwardKinematics(self.q)
            error = self.compute_pose_error(self.current_pose, self.goal_pose)
            if time.time() - break_out_time > 10:
                print("10 seconds have passed! Breaking out of homing")
                pub.publish(joint_states)
                break_out = True

        # Wait for the robot to settle after homing
        rospy.sleep(1.5)
        print("Starting goal tracking")
        rospy.sleep(2)
        # Initialize the joint angles and Jacobian matrix
        self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
        self.goal_pose = ur5FK.ForwardKinematics(self.q)
        self.J = ur5FK.Jacobian(self.q)

        # Enable the ready_for_mediapipe_input flag to indicate that the goal poses are ready to be published from video feed
        self.ready_for_mediapipe_input = True
        while not rospy.is_shutdown():
            # Check the message is acceptable
            if len(self.joint_positions)>0:
                # Compute the current end effector pose
                self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
                self.prev_pose = self.current_pose
                self.current_pose = ur5FK.ForwardKinematics(self.q)
                # Compute the error between the current and goal poses - remember goal poses are updated by the callback function transform_callback
                error = self.compute_pose_error(self.current_pose, self.goal_pose)
                
                # Check if the error is greater than the convergence threshold
                if np.linalg.norm(error) > convergence_threshold:
                    joint_states.data = self.iterative_ik_solver(error)
                    # if we have not reached the goal poses yet, publish the joint states. In other words move the robot
                    pub.publish(joint_states)

            # Build the transformation message to display a homogeneous transformation version of what the robot is doing
            transform.header.stamp = rospy.Time.now()
            # Assign translation
            transform.transform.translation.x = self.goal_pose[0, 3]
            transform.transform.translation.y = self.goal_pose[1, 3]
            transform.transform.translation.z = self.goal_pose[2, 3]

            # Convert rotation matrix to quaternion
            quaternion = tf.quaternion_from_matrix(self.goal_pose)
            # Create a Quaternion message and assign values
            transform.transform.rotation = Quaternion(*quaternion)

            # Publish the transform
            tf_broadcaster.sendTransform(transform)

            # Control the gripper based on the gripper state from the /gripper_condition topic in the callback function gripper_callback
            if self.gripper_state == 0:
                gripper_pub.publish(Float64(100))
                print("Gripper Open")
            elif self.gripper_state == 2:
                gripper_pub.publish(Float64(-100))
                print("Gripper Closed")
            # Sleep for the loop rate for control
            rate.sleep()



if __name__ == '__main__':
 
    try:
        IT_IK_Object = Iter_IK_ur5()
        IT_IK_Object.publisher()
    except rospy.ROSInterruptException:
        pass


