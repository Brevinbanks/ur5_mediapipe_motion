#!/usr/bin/env python

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
        self.joint_positions = []
        self.current_pose = np.eye(4)
        self.prev_pose = np.eye(4)
        self.J = np.eye(6)
        self.J_prev = np.eye(6)
        self.dq = np.array([[0],[1],[0],[0],[0],[0]])
        self.q = np.array([[0],[0],[0],[0],[0],[0]])
        self.goal_pose = np.eye(4)
        self.ready_for_mediapipe_input = False
        self.gripper_state = 0

    def ForwardKinematics(self, q1, q2, q3, q4, q5, q6):
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
        
        # E = np.round(E, decimals=4)

        return E

    def Jacobian(self, q1, q2, q3, q4, q5, q6):

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


        # J = np.round(J, decimals=4)

        return J



    # Implement the iterative IK solver
    def iterative_ik_solver(self, error):
        
        # Initialize joint angles (q) to some initial guess
        q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
                      
        
        # Set maximum number of iterations and convergence threshold
        # max_iterations = 100
        # convergence_threshold = 0.001
        
        # for iteration in range(max_iterations):

        # Compute current end effector pose using forward kinematics
        # current_pose = ForwardKinematics(g[0],g[1],g[2],g[3],g[4],g[5])


        # Compute error between current and goal poses
        # error = compute_pose_error(start_pose, goal_pose)
        
        # Check for convergence
        # if np.linalg.norm(error) < convergence_threshold:
        #     break
        Dmax = .2
        p_error = error
        error = self.clamp_mag(error,Dmax)
        
        # Compute Jacobian matrix
        self.J = ur5FK.Jacobian(self.q)
        # print("Singluarity: ", np.linalg.det(self.J),"Error: ",p_error,"E norm: " ,np.linalg.norm(p_error)," Clamped Error N: ",np.linalg.norm(error))
        k = 1
        # dx = self.current_pose-self.prev_pose
        dx = self.compute_pose_error(self.prev_pose, self.current_pose)
        # self.J_prev = self.J
        # self.J = self.J_prev + np.outer((np.subtract(dx,np.dot(self.J_prev,self.dq).T)/(np.linalg.norm(self.dq)**2)),self.dq)

        # I = np.eye(6)  # Identity matrix
        # delta_J = self.J - self.J_prev   # Change in Jacobian matrix
        # delta_dx = dx - np.dot(self.J, delta_J.T)  # Change in end-effector position or velocity

        # # Update the Jacobian matrix using Broyden's method
        # updated_J = self.J + np.dot(np.dot(delta_dx,delta_J), np.linalg.inv(np.dot(delta_J.T, delta_J) + I))

        # self.dq = np.dot(np.linalg.pinv(self.J),error)
        # print(self.J)
        lambda_damp = 0.01
        # Update joint angles using Jacobian pseudo-inverse
        # dq = k*np.dot(np.linalg.pinv(J),error)
        # error[3] =0
        # error[4] =0
        # error[5] =0
        K = np.diag([5,5,5,1,1,1])
        # np.dot(np.linalg.inv(np.dot(J.T , J) + damping_factor**2 * np.eye(6)), J.T)
        # Update joint angles using the damped-least squares method
        self.dq = np.dot(np.dot(np.linalg.inv(np.dot(self.J.T , self.J) + lambda_damp**2 * np.eye(6)), self.J.T),np.dot(K,error))
        newq = q
        newq += self.dq
        alpha = 0.9
        # Apply the low-pass filter inline
        self.q =  alpha * newq + (1 - alpha) * self.q

        # print(self.J)
            
        return self.q



    def clamp_mag(self, error,Dmax):
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
        # rospy.loginfo('[GRIPPER STATE RECEIVED] %d', msg.data)
        self.gripper_state = msg.data

    def publisher(self):
        rospy.Subscriber('/mediapipe_transform', TransformStamped, self.transform_callback)
        rospy.Subscriber('/gripper_condition', Int32, self.gripper_callback)

        rospy.init_node('joint_states_publisher', anonymous=True)
        pub = rospy.Publisher('cv_des_joint_states', Float64MultiArray, queue_size=10)

        rospy.Subscriber('joint_states', JointState, self.ur5_joint_sub_callback)
        convergence_threshold = 0.001

        gripper_pub = rospy.Publisher('/Slider1_effort_controller/command', Float64, queue_size=10)
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        transform = geometry_msgs.msg.TransformStamped()

        
        transform.header.frame_id = "base_link"  # Assuming models are defined in the base_link frame
        transform.child_frame_id = "ik_goal"

        parser = ur5_urdf_parser()


        # rospy.init_node('gripper_controller', anonymous=True)
        # pub_grip = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)
        rate = rospy.Rate(10)  # 10Hz
        
        # gripper_state_msg = JointState()
        # gripper_state_msg.name = ['finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint']
            
        self.goal_pose = np.array([[-1.00000000e+00, -9.79311649e-12, -4.89652763e-12, 8.17250000e-01]
                              ,[-4.89652763e-12, 2.99829594e-28, 1.00000000e+00, 1.91450000e-01]
                              ,[-9.79311649e-12, 1.00000000e+00, -4.79525653e-23, -5.49100000e-03]
                              ,[0.0, 0.0, 0.0, 1.0]])
        
        
        rospy.sleep(1)
        joint_states = Float64MultiArray()
        # joint_states.data = [-1.57,-1.45,2.01,-2.07,-1.57,-1.57]
        joint_states.data = [-1.91, -1.4, 2.15, -2.25, -1.57, -0.38]   # Modify with desired values

        pub.publish(joint_states)
        error = 5
        print("putting robot in position")
        break_out = False
        break_out_time = time.time()
        while np.linalg.norm(error)>1 and break_out == False:
            self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
            self.goal_pose = ur5FK.ForwardKinematics([-1.57,-1.45,2.01,-2.07,-1.57,-1.57])
            self.current_pose = ur5FK.ForwardKinematics(self.q)
            error = self.compute_pose_error(self.current_pose, self.goal_pose)
            if time.time() - break_out_time > 10:
                print("10 seconds have passed! Breaking out of homing")
                pub.publish(joint_states)
                break_out = True

        rospy.sleep(1.5)
        print("Starting goal tracking")
        rospy.sleep(2)
        self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
        self.goal_pose = ur5FK.ForwardKinematics(self.q)
        self.J = ur5FK.Jacobian(self.q)
        start_time = time.time()
        dir = 1
        self.ready_for_mediapipe_input = True
        while not rospy.is_shutdown():
            # self.goal_pose[0,3] = self.goal_pose[0,3]-0.01
            # Define the array of joint states
            
            # In here perform CV work to find the required joint angles
            #add velocity limiter, joint range threshold/limits
            # print(len(self.joint_positions))
            if len(self.joint_positions)>0:
                current_time = time.time()
                
                self.q = np.array([self.joint_positions[3],self.joint_positions[2],self.joint_positions[1],self.joint_positions[4],self.joint_positions[5],self.joint_positions[6]])
                self.prev_pose = self.current_pose
                self.current_pose = ur5FK.ForwardKinematics(self.q)
                # self.goal_pose = self.current_pose#self.stick_up_down(dir, self.q,self.goal_pose,self.current_pose)
                error = self.compute_pose_error(self.current_pose, self.goal_pose)
                

               
                if np.linalg.norm(error) > convergence_threshold:
                    joint_states.data = self.iterative_ik_solver(error) # Modify with desired values

                    # Get the forward kinematics
                    # pose = kinematics.forward(q)
                    
                    # # Get the Jacobian
                    # Jacobian = kinematics.jacobian(q)
                    #[shoulder pan joint, shoulder lift joint, elbow joint, wrist 1 joint, wrist 2 joint, wrist 3 joint]
                # joint_states.data = [-2.41, -0.94, 2.37, -2.88, 0.06, -0.07]   # Modify with desired values
                    # joint_states.data = [0, 0, 0, 0.0, 0, 0.0] 
                    # print(self.joint_positions)
                # Publish the joint states
                    pub.publish(joint_states)
            
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

            # Control the gripper
            
            # gripper_state_msg.header.stamp = rospy.Time.now()
            # gripper_state_msg.position = [0.8, 0.8, -0.8, 0.8, 0.8, -0.8] # 0 to 0.8 are acceptable values for the gripper
            if self.gripper_state == 0:
                gripper_pub.publish(Float64(100))
            elif self.gripper_state == 2:
                gripper_pub.publish(Float64(-100))

            print(self.gripper_state)
            rate.sleep()
            # rospy.spinOnce()


    def stick_up_down(self,dir,q,old_goal,current_pose):
        theta = q[0]
        goal_pose = old_goal
        goal_pose[0,3] = goal_pose[0,3]+np.cos(theta)*dir*0.02
        goal_pose[1,3] = goal_pose[1,3]+np.sin(theta)*dir*0.02
        if np.linalg.norm(current_pose[0:3,3])> 0.80 and np.linalg.norm(goal_pose[0:3,3])>=np.linalg.norm(current_pose[0:3,3]):
            goal_pose[0:3,3] = current_pose[0:3,3]
            rospy.loginfo("Arm inner limit reached at "+str(np.linalg.norm(self.current_pose[0:3,3]))+"m normal from the robot base_link to tool0")
        if np.linalg.norm(current_pose[0:3,3])< 0.40 and np.linalg.norm(goal_pose[0:3,3])<=np.linalg.norm(current_pose[0:3,3]):
            goal_pose[0:3,3] = current_pose[0:3,3]
            rospy.loginfo("Arm outer limit reached at "+str(np.linalg.norm(self.current_pose[0:3,3]))+"m normal from the robot base_link to tool0")
        return goal_pose

if __name__ == '__main__':
 
    try:
        IT_IK_Object = Iter_IK_ur5()
        IT_IK_Object.publisher()
    except rospy.ROSInterruptException:
        pass


