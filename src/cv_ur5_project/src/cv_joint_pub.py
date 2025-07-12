#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

def publisher():
    rospy.init_node('joint_states_publisher', anonymous=True)
    pub = rospy.Publisher('cv_des_joint_states', Float64MultiArray, queue_size=10)
    # rospy.init_node('gripper_controller', anonymous=True)
    pub_grip = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    
    gripper_state_msg = JointState()
    gripper_state_msg.name = ['finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint']
        
    while not rospy.is_shutdown():
        # Define the array of joint states
        joint_states = Float64MultiArray()
        # In here perform CV work to find the required joint angles
        #add velocity limiter, joint range threshold/limits
        
        joint_states.data = [-1.91, -1.4, 2.15, -2.25, -1.57, -0.38]   # Modify with desired values

        # joint_states.data = [-2.41, -0.94, 2.37, -2.88, 0.06, -0.07]   # Modify with desired values
        # Publish the joint states
        pub.publish(joint_states)



        # Control the gripper
        
        gripper_state_msg.header.stamp = rospy.Time.now()
        gripper_state_msg.position = [0.8, 0.8, -0.8, 0.8, 0.8, -0.8] # 0 to 0.8 are acceptable values for the gripper
        pub_grip.publish(gripper_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

