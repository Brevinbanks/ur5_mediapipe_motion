"""
Mediapipe + ROS Integration with GUI

This script captures live video, detects body pose and hand landmarks using Mediapipe,
computes a transformation matrix based on right wrist and hand orientation, then publishes:
  - A 6DOF transform (`/mediapipe_transform`) from a virtual frame to the robot base
  - A gripper control signal (`/gripper_condition`) based on finger bending

Additionally, it launches a GUI that displays:
  - Live annotated camera feed
  - Tool position in robot frame
  - Orientation as axis-angle
  - Gripper condition (0 = open, 2 = closed)

Author: Brevin Banks, Stefan Hustrulid, and Baldur Hua
Date: 2024
"""
### MAKE SURE TO CHANGE THE ROS MASTER IP ADDRESS BELOW TO YOUR OWN MACHINE'S IP ADDRESS IN FUNCTION run_vision_ros() ###
### THEN ENSURE THE WEB STOCKET SERVER IS RUNNING ON YOUR MACHINE AT PORT 9090 AND THE ROS MASTER IS RUNNING ###

import tkinter as tk
from PIL import Image, ImageTk
import threading
import numpy as np
import cv2
import mediapipe as mp
import roslibpy
from transforms3d.quaternions import mat2quat, quat2axangle

# Landmark indices
R_Index = 20
R_Pinky = 18
R_Wrist = 16
R_Shoulder = 12
R_Hip = 24
H_Wrist = 0
H_Index = 5
H_Pinky = 17
H_Index_Pip = 6
H_Middle = 9
H_Middle_Pip = 10
H_Ring = 13
H_Ring_Pip = 14

origin_options = ["default", "r_hip", "r_shoulder"]
origin_x = origin_options[0]
origin_y = origin_options[1]
origin_z = origin_options[0]
scaling = [1.4, 1.2, 0.8]

# Shared GUI state
state = {
    'frame': None,
    'position': np.zeros(3),
    'axis_angle': np.array([0, 0, 1, 0]),
    'gripper': 0
}

def HandWorld2RobotWorld(landmark, translation, scaling):
    R = np.array([
        [1, 0, 0],
        [0, np.cos(-np.pi/2), -np.sin(-np.pi/2)],
        [0, np.sin(-np.pi/2),  np.cos(-np.pi/2)]
    ])
    E = np.append(np.multiply(np.append(R, np.transpose([translation]), axis=1), np.transpose([scaling])),
                  np.array([[0, 0, 0, 1]]), axis=0)
    if len(landmark) == 3:
        landmark = np.append(landmark, 1)
    return np.dot(E, landmark)[0:3]

def publish_transform(E_tool, tf_pub):
    quat = mat2quat(E_tool[:3, :3])  # [w, x, y, z]
    msg = {
        'header': {'frame_id': 'base_link', 'stamp': {'secs': 0, 'nsecs': 0}},
        'child_frame_id': 'media_frame',
        'transform': {
            'translation': {'x': float(E_tool[0, 3]), 'y': float(E_tool[1, 3]), 'z': float(E_tool[2, 3])},
            'rotation': {'w': float(quat[0]), 'x': float(quat[1]), 'y': float(quat[2]), 'z': float(quat[3])}
        }
    }
    tf_pub.publish(roslibpy.Message(msg))

def publish_gripper_state(state_val, gripper_pub):
    gripper_pub.publish(roslibpy.Message({'data': int(state_val)}))

def run_vision_ros():
    client = roslibpy.Ros(host='192.168.127.174', port=9090) #<-- Change to your ROS master IP
    client.run()

    tf_pub = roslibpy.Topic(client, '/mediapipe_transform', 'geometry_msgs/TransformStamped')
    gripper_pub = roslibpy.Topic(client, '/gripper_condition', 'std_msgs/Int32')

    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose
    mp_hands = mp.solutions.hands

    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    cap = cv2.VideoCapture(0)
    Basis = np.eye(3)
    tool_pos = np.zeros([3])
    behavior_key = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(frame_rgb)

        if not pose_results.pose_world_landmarks:
            continue

        hip_pos = pose_results.pose_world_landmarks.landmark[R_Hip]
        wrist_pos = pose_results.pose_world_landmarks.landmark[R_Wrist]
        translation = [0, 0, 0]
        if origin_x == "r_hip":
            translation[0] = hip_pos.x
        if origin_y == "r_hip":
            translation[1] = hip_pos.y
        if origin_z == "r_hip":
            translation[2] = hip_pos.z

        tool_pos = HandWorld2RobotWorld([wrist_pos.x, wrist_pos.y, wrist_pos.z], translation, scaling)

        hand_results = hands.process(frame_rgb)
        if hand_results.multi_handedness:
            for idx, hand in enumerate(hand_results.multi_handedness):
                if hand.classification[0].label == "Left":
                    landmarks = hand_results.multi_hand_world_landmarks[idx].landmark

                    # Rotation
                    h_wrist = HandWorld2RobotWorld([landmarks[H_Wrist].x, landmarks[H_Wrist].y, landmarks[H_Wrist].z], translation, [1, 1, 1])
                    h_index = HandWorld2RobotWorld([landmarks[H_Index].x, landmarks[H_Index].y, landmarks[H_Index].z], translation, [1, 1, 1])
                    h_pinky = HandWorld2RobotWorld([landmarks[H_Pinky].x, landmarks[H_Pinky].y, landmarks[H_Pinky].z], translation, [1, 1, 1])

                    Index_Dir = h_index - h_wrist; Index_Dir /= np.linalg.norm(Index_Dir)
                    Pinky_Dir = h_pinky - h_wrist; Pinky_Dir /= np.linalg.norm(Pinky_Dir)
                    Palm_Dir = np.cross(Index_Dir, Pinky_Dir); Palm_Dir /= np.linalg.norm(Palm_Dir)
                    Thumb_Dir = np.cross(Index_Dir, Palm_Dir); Thumb_Dir /= np.linalg.norm(Thumb_Dir)
                    Basis = np.stack([Thumb_Dir, Index_Dir, Palm_Dir], axis=1)

                    def norm_dir(a, b):
                        d = a - b
                        return d / np.linalg.norm(d)

                    index_dir = norm_dir(
                        HandWorld2RobotWorld([landmarks[H_Index].x, landmarks[H_Index].y, landmarks[H_Index].z], translation, [1, 1, 1]),
                        HandWorld2RobotWorld([landmarks[H_Index_Pip].x, landmarks[H_Index_Pip].y, landmarks[H_Index_Pip].z], translation, [1, 1, 1])
                    )
                    middle_dir = norm_dir(
                        HandWorld2RobotWorld([landmarks[H_Middle].x, landmarks[H_Middle].y, landmarks[H_Middle].z], translation, [1, 1, 1]),
                        HandWorld2RobotWorld([landmarks[H_Middle_Pip].x, landmarks[H_Middle_Pip].y, landmarks[H_Middle_Pip].z], translation, [1, 1, 1])
                    )
                    ring_dir = norm_dir(
                        HandWorld2RobotWorld([landmarks[H_Ring].x, landmarks[H_Ring].y, landmarks[H_Ring].z], translation, [1, 1, 1]),
                        HandWorld2RobotWorld([landmarks[H_Ring_Pip].x, landmarks[H_Ring_Pip].y, landmarks[H_Ring_Pip].z], translation, [1, 1, 1])
                    )

                    wrist_dir = norm_dir(h_wrist, h_index)
                    Index_Metric = np.dot(wrist_dir, index_dir)
                    Middle_Metric = np.dot(wrist_dir, middle_dir)
                    Ring_Metric = np.dot(wrist_dir, ring_dir)

                    Index_Vote = int(Index_Metric <= 0.7)
                    Middle_Vote = int(Middle_Metric <= 0.7)
                    Ring_Vote = int(Ring_Metric <= 0.7)
                    behavior_key = 0 if (Index_Vote + Middle_Vote + Ring_Vote) < 2 else 2

        E_tool = np.eye(4)
        E_tool[:3, :3] = Basis
        E_tool[:3, 3] = tool_pos
        publish_transform(E_tool, tf_pub)
        publish_gripper_state(behavior_key, gripper_pub)

        quat = mat2quat(Basis)
        axis, angle = quat2axangle(quat)
        state['position'] = tool_pos
        state['axis_angle'] = np.append(axis, np.degrees(angle))
        state['gripper'] = behavior_key
        mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        state['frame'] = frame.copy()

    cap.release()
    tf_pub.unadvertise()
    gripper_pub.unadvertise()
    client.terminate()

def launch_gui():
    root = tk.Tk()
    root.title("ROS Mediapipe GUI")
    root.attributes('-topmost', True)
    root.resizable(False, False)
    # Keep the GUI on top
    # root.attributes('-topmost', True)

    video_label = tk.Label(root)
    video_label.grid(row=0, column=0, columnspan=2)

    pos_label = tk.Label(root, text="Position:")
    pos_val = tk.Label(root, text="")
    rot_label = tk.Label(root, text="Rotation:")
    rot_val = tk.Label(root, text="")
    grip_label = tk.Label(root, text="Gripper:")
    grip_val = tk.Label(root, text="")

    pos_label.grid(row=1, column=0, sticky='w')
    pos_val.grid(row=1, column=1, sticky='w')
    rot_label.grid(row=2, column=0, sticky='w')
    rot_val.grid(row=2, column=1, sticky='w')
    grip_label.grid(row=3, column=0, sticky='w')
    grip_val.grid(row=3, column=1, sticky='w')

    def update_gui():
        if state['frame'] is not None:
            img = Image.fromarray(cv2.cvtColor(state['frame'], cv2.COLOR_BGR2RGB))
            img = img.resize((640, 480))
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)

        p = state['position']
        a = state['axis_angle']
        pos_val.config(text=f"{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}")
        rot_val.config(text=f"[{a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}], {a[3]:.1f}°")
        grip_val.config(text=str(state['gripper']))
        root.after(100, update_gui)

    update_gui()
    root.mainloop()

# Start threads
threading.Thread(target=run_vision_ros, daemon=True).start()
launch_gui()


