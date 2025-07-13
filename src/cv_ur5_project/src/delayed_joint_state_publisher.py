#!/usr/bin/env python

# This ROS node publishes an initial joint state message (retrieved from the ROS parameter server)
# to the /joint_states topic at 10 Hz, after a short delay.
# Useful for bootstrapping simulations where joint state data must be published right after launch.
# Note: initial_joint_states here is loaded via rospy.get_param, but it's being used like a JointState object. 
# That'll cause a bug unless you’re explicitly storing a full JointState message structure as a parameter — which is unusual. 
# If you see a TypeError about missing fields or attributes like header.stamp, you'll need to manually 
# construct the JointState from the param dict.

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def main():
    rospy.init_node('delayed_joint_state_publisher')

    # Delay startup to ensure parameters are available
    time.sleep(1)

    # Load joint state from the ROS parameter server (must be preloaded into /joint_states param)
    initial_joint_states = rospy.get_param('/joint_states')

    # Set up publisher for /joint_states
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Update timestamp before publishing
        initial_joint_states.header.stamp = rospy.Time.now()
        pub.publish(initial_joint_states)
        rate.sleep()

if __name__ == '__main__':
    main()
