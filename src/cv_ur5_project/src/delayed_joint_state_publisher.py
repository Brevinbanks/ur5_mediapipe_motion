#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def main():
    rospy.init_node('delayed_joint_state_publisher')

    # Wait for 1 second
    time.sleep(1)

    # Load initial joint states
    initial_joint_states = rospy.get_param('/joint_states')

    # Publish initial joint states
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        initial_joint_states.header.stamp = rospy.Time.now()
        pub.publish(initial_joint_states)
        rate.sleep()

if __name__ == '__main__':
    main()
