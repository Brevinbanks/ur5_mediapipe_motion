#!/usr/bin/env python

# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Header

# def move_ur5(joint_angles):
#     rospy.init_node('brevin_ur5_controller', anonymous=True)
#     pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
#     rate = rospy.Rate(10)  # 10Hz

#     while not rospy.is_shutdown():
#         # Create a JointTrajectory message
#         trajectory_msg = JointTrajectory()
#         trajectory_msg.header = Header()
#         trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

#         # Create a JointTrajectoryPoint for the desired joint angles
#         point = JointTrajectoryPoint()
#         point.positions = joint_angles
#         point.time_from_start = rospy.Duration(1)  # Adjust the time_from_start as needed

#         # Add the trajectory point to the JointTrajectory message
#         trajectory_msg.points.append(point)

#         # Publish the JointTrajectory message
#         pub.publish(trajectory_msg)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         # Example joint angles: [joint1, joint2, joint3, joint4, joint5, joint6]
#         joint_angles = [0.0, -0.9, 0.7, -1.3, -1.57, 0.]  # Modify with desired joint angles
#         move_ur5(joint_angles)
#     except rospy.ROSInterruptException:
#         pass

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header

def callback(data):
    # Extract the first 6 values as joint angles
    joint_angles = data.data[:6]
    
    # Process the joint angles and control the UR5 robot
    # Example: move_ur5(joint_angles)
    # print("Received joint angles:", joint_angles)
    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.header = Header()
    trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Create a JointTrajectoryPoint for the desired joint angles
    point = JointTrajectoryPoint()
    point.positions = joint_angles
    point.time_from_start = rospy.Duration(1)  # Adjust the time_from_start as needed

    # Add the trajectory point to the JointTrajectory message
    trajectory_msg.points.append(point)

    # Publish the JointTrajectory message
    pub.publish(trajectory_msg)

def move_ur5():
    rospy.init_node('ur5_controller', anonymous=True)
    rospy.Subscriber('cv_des_joint_states', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # rospy.init_node('brevin_ur5_controller', anonymous=True)
        pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        move_ur5()
    except rospy.ROSInterruptException:
        pass