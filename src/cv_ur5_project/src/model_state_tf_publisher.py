#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import tf2_ros
import geometry_msgs.msg

# Dictionary to store the latest pose of each model
model_poses = {}

def model_states_callback(msg):
    global model_poses
    model_poses = {name: pose for name, pose in zip(msg.name, msg.pose)}

def publish_tf_frames():
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    for name, pose in model_poses.items():
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"  # Assuming models are defined in the world frame
        transform.child_frame_id = name
        transform.transform.translation = pose.position
        transform.transform.rotation = pose.orientation

        tf_broadcaster.sendTransform(transform)

def main():
    rospy.init_node('model_state_tf_publisher')

    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    rate = rospy.Rate(30)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        publish_tf_frames()
        rate.sleep()

if __name__ == '__main__':
    main()
