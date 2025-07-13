#!/usr/bin/env python

# This ROS node publishes a predefined array of 6 joint values to the 'cv_des_joint_states' topic
# at 10 Hz. It is used to simulate or command a gripper or robotic hand in a test/demo environment.
# MAKE SURE ur5_controller.py IS RUNNING BEFORE RUNNING THIS SCRIPT

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

def publisher():
    rospy.init_node('joint_states_publisher', anonymous=True)

    # Publisher to send joint states as a Float64MultiArray
    pub = rospy.Publisher('cv_des_joint_states', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz loop rate

    # Declare gripper joint names (not used directly in publishing but could be extended)
    gripper_state_msg = JointState()
    gripper_state_msg.name = [
        'finger_joint',
        'left_inner_knuckle_joint',
        'left_inner_finger_joint',
        'right_outer_knuckle_joint',
        'right_inner_knuckle_joint',
        'right_inner_finger_joint'
    ]
        
    while not rospy.is_shutdown():
        # Create and fill the joint state message with 6 values
        joint_states = Float64MultiArray()
        joint_states.data = [-1.91, -1.4, 2.15, -2.25, -1.57, -0.38]  # Replace with live or desired values 

        # Publish joint state array
        pub.publish(joint_states)

        # Sleep to maintain loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
