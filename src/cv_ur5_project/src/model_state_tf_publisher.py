#!/usr/bin/env python

# This ROS node listens to Gazebo's /gazebo/model_states topic and publishes static TF frames
# for each model in the simulation. It makes every model in Gazebo visible to the TF tree
# using their current pose in the world frame.

import rospy
from gazebo_msgs.msg import ModelStates
import tf2_ros
import geometry_msgs.msg

# Dictionary to store the latest pose of each model
model_poses = {}

def model_states_callback(msg):
    # Update the dictionary with the latest poses of all models
    global model_poses
    model_poses = {name: pose for name, pose in zip(msg.name, msg.pose)}

def publish_tf_frames():
    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    for name, pose in model_poses.items():
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"  # All poses are relative to the "world" frame
        transform.child_frame_id = name
        transform.transform.translation = pose.position
        transform.transform.rotation = pose.orientation

        tf_broadcaster.sendTransform(transform)

def main():
    # Initialize the ROS node
    rospy.init_node('model_state_tf_publisher')

    # Subscribe to the model states published by Gazebo
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    # Publish TF frames at a fixed rate
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        publish_tf_frames()
        rate.sleep()

if __name__ == '__main__':
    main()
