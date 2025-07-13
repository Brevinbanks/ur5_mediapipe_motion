# ur5_mediapipe_motion
ROS + Mediapipe Integration for Real-Time Hand Tracking and Robot Control Publishes 3D transforms and gripper states from Mediapipe pose and hand landmarks to ROS.


This Python tool uses Mediapipe to detect body pose and hand landmarks in real time from a webcam feed. It converts wrist and finger data into a 6DOF pose, publishing it as a transform to a ROS system and controlling a ur5 robot with kinematics mapped to the user's right hand with their right hip as a base and a gripper based on finger curl. A simple GUI shows live video, pose, orientation, and gripper state. On the ROS server, Gazebo is used to simulate a table, block, and camera with pick and place block physics. Rviz is used to visualize the robot and the world and frame specific data. 

Alternativley, you can use the rqt GUI to control the robot with the mouse.

# Installation:
This project was developed and tested on Ubuntu 18.04 with ROS Melodic.
Ensure you have ROS Melodic or your compatible distribution with the right OS installed on your machine. If not, follow the instructions here: https://www.ros.org/install/

Clone this repository into your catkin workspace:

```bash
cd ~/cv_workspace/src
git clone https://github.com/Brevinbanks/ur5_mediapipe_motion.git
```

Move the python3files folder out of the workspace and to a separate working directory for python3 and mediapipe files.

# Setup Instructions for cv_ur5_project Workspace

## ROS Dependencies

Run these commands to install all required ROS packages:

```bash
sudo apt update
sudo apt install -y \
  ros-melodic-actionlib \
  ros-melodic-control-toolbox \
  ros-melodic-cv-bridge \
  ros-melodic-dynamic-reconfigure \
  ros-melodic-gazebo-ros \
  ros-melodic-geometry-msgs \
  ros-melodic-image-transport \
  ros-melodic-robot-state-publisher \
  ros-melodic-roboticsgroup-gazebo-plugins \
  ros-melodic-roscpp \
  ros-melodic-rqt-joint-trajectory-controller \
  ros-melodic-sensor-msgs \
  ros-melodic-std-msgs \
  ros-melodic-tf \
  ros-melodic-ur-description \
  ros-melodic-visualization-msgs
```
Note: Replace `melodic` with your ROS distribution name.

## A Note on Python Versions and Mediapipe
Melodic and ROS1 use Python 2.7 for workspace scripts and dependencies.
However Mediapipe is only compatible with Python 3.6+.
Because of this we have two paths forward. If you are using Linux as your base OS, you can use the `python3` command to run the Mediapipe scripts (assumin you have python3 installed), and if you've setup your workspace correctly then the ROS scripts should run under Python 2.7 just fine.
Google's Mediapipe should install via "pip install mediapipe" on most modern systems, but heads up:

    It officially supports Python 3.7 to 3.10 (sometimes 3.11 works but not guaranteed).

    You’ll need a 64-bit OS.

    On Linux, it can be a bit finicky—sometimes you need extra dependencies like libprotobuf or a proper C++ compiler toolchain installed.

    GPU support is more complicated; the default pip install is CPU-only.

Additionally, If you are operating with a VM for Linux you will need to take advantage of the websocket bridge to communicate with ROS across the VM from windows on python3. You may need to do this even if both python platforms are in Linux. I'll discuss how to do this in more detail in the next section.

## Python Dependencies
In your Python 2.7 environment, install numpy and sympy:
```bash
pip install numpy
pip install sympy
```
These are the only depenedencies for the project that may not already be installed in the ROS Melodic distribution.

## Mediapipe Dependencies
In your Python 3.6 environment, install the requirements.txt file in the python3files folder:
```bash
pip install -r requirements.txt
```
- assuming the requirements file is in the current folder of your terminal directory. (Not inside the cv_workspace folder)


## Build the Project
To build the project, navigate you terminal within the top level of the cv_workspace. Run the following commands in your terminal:

```bash
cd ~/cv_workspace
catkin_make
```
Anytime you reopen the terminal, you can run the following command in the top level of the cv_workspace to source the workspace so scripts and ROS handles are available:
```bash
source devel/setup.bash
```
# Running the Project
A good test launch file is  ur5_robotiq.launch. This will launch the ur5 robot, the rviz visualization, and the joint controller GUI. It won't start the Gazebo simulation, but it will start the rqt_reconfigure GUI which allows us to do a quick test of our environment.
```bash
cd ~/cv_workspace
roslaunch cv_ur5_project ur5_robotiq.launch
```
Rviz will open up with a ur5, some frames marked and floating, and a camera feed - but the feed will be black. This is because we haven't launched the Gazebo simulation yet.
Additionally, two rqt windows will open up. One is the joint controller GUI, and the other is the rqt_reconfigure GUI.

The joint controller will allow you to control the joints of the robot individually if you select the controller availalbe in the two drop down menus and click the play button. NOTE that you must turn the button back off to use the mediapipe controller or any other script based controller.

The rqt_reconfigure GUI will allow you to edit specific node parameters, but the default one that pops up is Slider1. This is the joint effort controller for the gripper. Click the drop down arrow next to the Slider1 item and then you can see a number for the effort.
-100 is closed, 100 is open. Check the box to have these changes effect the gripper. Uncheck the box if you want to have the mediapipe controller control the gripper.

Be sure to kill the launch process if you want to launch a different file.

To run the main project after building and sourceing (assuming you have 0 hair ripping ROS errors to fix), you can launch the project with the following command:

```bash
cd ~/cv_workspace
roslaunch cv_ur5_project ur5_cv_control.launch
```
A few windows will pop up. An Rviz window will open up with a ur5, some frames marked and floating, and a camera feed from gazebo.
Gazebo will also start up and you can see the robot and table in the world. There will be a blue cube that sits on the table, but occasionally it will fall off the table if the cube is loaded in too fast. Restart the simulation if this happens.
Again, two rqt windows will open up. One is the joint controller GUI, and the other is the rqt_reconfigure GUI just like before.
We won't need to use them unless we want to debug the robot in the Gazebo simulation.

The Gazebo simulation is used to help simulate collision physics between the robot and other objects in the world.

Before we can control the robot with any outside messages, we need the robot controller to be running. We can do this with the following command:
```bash
cd ~/cv_workspace/src/cv_ur5_project/src
./ur5_controller.py
```
This will start the joint state publisher and the robot controller. The joint state publisher will publish the current joint positions to the /joint_states topic. The robot controller will subscribe to the /arm_controller/follow_joint_trajectory topic and will move the robot to the goal poses. REMEMBER ./ur5_controller.py IS REQUIRED FOR THE ROBOT TO MOVE VIA SCRIPT.

We have two scripts that interact with the robot controller. One is the cv_joint_pub.py script, and the other is the kinematic_control_ur5.py script. The cv_joint_pub.py script is a simple script that will move the robot to a goal pose based on given joint angles in the script. The kinematic_control_ur5.py script will take in a list of joint angles and will move the robot to the goal pose and then move the robot based on input from the mediapipe pose and hand landmarks.

To run the basic cv_joint_pub.py script, we can run the following command:
```bash
cd ~/cv_workspace/src/cv_ur5_project/src
./ cv_joint_pub.py
```
The robot should move to the joint angles in the script.
Kill that script with ctrl+c in the terminal.

To run the kinematic_control_ur5.py script, we can run the following command:
```bash
cd ~/cv_workspace/src/cv_ur5_project/src
./ kinematic_control_ur5.py
```
The robot should move to the home pose in the script and then move to the goal poses based on the mediapipe pose and hand landmarks.
However, the robot will not move to the goal poses if the mediapipe pose and hand landmarks are not detected or the mediapipe script is not running. Let's stop the script with ctrl+c in the terminal for now.

# Control the Robot with the Mediapipe Controller
Make sure rosbridge_websocket is running on your ROS machine. This is a ROS package that allows you to communicate with ROS from a web browser or other machines via websockets.
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

Now we can talk to the ROS machine from python3 across the VM or python versions as long as the python script we run connects itselft the IP Address of the ROS machine.
In the ur5_arm_vision_2_trans_and_grip_python3_talker.py (python3files folder) change the IP address to your ROS machine's IP address.
This is in the run_vision_ros() function.
This looks like: client = roslibpy.Ros(host='192.168.127.174', port=9090)

You can find the host IP address by running hostname -I in the terminal.
```bash
hostname -I

```

We can then run the python script with the terminal in our python3files folder with the following command:
```bash
python3 ur5_arm_vision_2_trans_and_grip_python3_talker.py
```

A TKinter GUI will pop up with a live video feed from the camera, and previews of the right hand pose relative to the right hip of whoever is in front of the camera. If no one is in front of the camera nothing should be output, but it's possible mediapipe may falsely detect a hand if it is not in front of the camera. You can test this by moving in front of the camera so that your right arm and hips are visible. 

While not shown in the GUI, Mediapipe is also tracking finger positions on the hand. It uses heuristics to determine if the hand is closed or open for controlllng the gripper.

# Mediapipe

Google's Mediapipe is a framework for building machine learning models that are used to detect and track objects in real-time. It's especially well made for determining face, pose, and hand positions, orientations, features, etc. It is designed to work with a variety of different computer vision models, including object detection, face detection, and hand tracking. Mediapipe is used in many applications, including self-driving cars, robotics, and augmented reality.
Learn more about Mediapipe here: https://mediapipe.dev/ and https://chuoling.github.io/mediapipe/

# UR5 Robot

The UR5 robot is a popular robotic arm that is commonly used in robotics and industrial automation. It is a 6-axis robot with a gripper that can be controlled by a variety of different software. The UR5 robot is often used in industrial applications, such as assembly line automation, material handling, and warehouse automation. Learn more about the UR5 robot here: https://www.universal-robots.com/products/ur5-robot/

The forward kinematics, jacobian, and inverse kinematics of the UR5 robot are used to calculate the end effector pose and the robot's joint positions. The solutions for forward kinematics and inverse kinematics are based on the robot's kinematics, which are defined in the URDF file, were all solved by me using gemoetric twists, exponential matricies, and the jacobian inverse. 
Learn more about these basic robotics concepts here: https://www.universal-robots.com/articles/ur-robot-kinematics/
Geometric twists: https://en.wikipedia.org/wiki/Twist_(mathematics)
Exponential matricies: https://en.wikipedia.org/wiki/Matrix_exponential
Jacobian inverse: https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant#Jacobian_matrix_inverse
Inverse kinematics: https://en.wikipedia.org/wiki/Inverse_kinematics

# UR5 Control via Inverse Kinematics - Broyden's Method
The UR5 robot is controlled via inverse kinematics - by means of broyden's method. This method is used to solve for the joint angles that will move the robot to a desired end effector pose. Broydens method is a numerical method that takes the jacobian inverse of the robot and multiplies it by the error between the desired end effector pose and the current end effector pose. It includes a damping term to reduce the oscillations of the joint angles and helps to converge to the desired pose. This makes for smoother and more stable control of the robot. Learn more abg
eout broyden's method here: 
https://en.wikipedia.org/wiki/Broyden%27s_method


# Tracking Motion with your Hand. Pick and Place
With the ur5_arm_vision_2_trans_and_grip_python3_talker.py file running, you are now sending homogeneous transformations to the robot via the /mediapipe_transform topic. This transform, relative to your right hip is mapped to the new goal for the robot's endeffector frame relative to the base frame. In other words, your right hip is the same as the robot' base frame. Rotating your body about your hip will rotate the base link, and moving your elbow and shoulder will have similiar effects on various joints of the robot. Because human arm length varies and is not at all the same as the length of the ur5 robot arm, scaling factors are used to make motion in the XYZ directions easier on the user. 

In ur5_arm_vision_2_trans_and_grip_python3_talker.py you will find a variable called "scaling" that has 3 values. These each are the scae factors for the X, Y, and Z directions that map your hand movement to the robot's movement. If the scale factor of X is 2, then for every unit of movement in the X direction, the robot will move 2 units.

With Gazebo open from the launch file, ur5_controller.py running, and kinematic_control_ur5.py running, you can now control the robot with the mediapipe controller. After the robot finishing the homing step from kinematic_control_ur5.py, pose detection will start and the robot will move to your hand position. The gripper will open and close to match the hand state. 

Try to move your hand to the blue cube in the Gazebo world and pick it up. Try to see if you can pick up the blue cube and place it in the white box. The robot does not have gaurds to stop it from hitting the table so be careful not to move to far or fast.

Thats the basics of the project. I hope you enjoyed it. If you have any questions, feel free to reach out to me on github or email me at my website https://brevinbanks.github.io/#contact

Special thanks to Stefan Hustrulid and Baldur Hua at Johns Hopkins University for their help with the Mediapipe integration. 

Stefan was responsible for the mediapipe pose tracking and robot frame integration.

Baldur was responsible for the gripper state tracking and finger heuristics.

# ROS Workspace File Structure Overview
There are two CMakeLists.txt files because:
•	One is at the root of your catkin workspace → for building the whole workspace
•	The other is inside each ROS package → for building that specific package

Files in tree under cv_workspace/src/
├── CMakeLists.txt-> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
└── cv_ur5_project-> Project name and directory
    ├── CMakeLists.txt-> Is a script that tells CMake how to build, link, and install your ROS package or C++ project.
    ├── config
    │   ├── arm_controller_ur5.yaml-> Controller to the ur5. Both a joint state and force controller.
    │   ├── gripper_controller.yaml-> Controller to the robotiq finger grip at the end of the ur5.A joint state and force controller.
    │   ├── gazebo_controller.yaml-> Controller to the robotiq finger grip for control within the Gazebo simulation.
    │   └── joint_state_controller.yaml-> Pushes ur5 Joint states to the ROS Server
    ├── cv_ur5_project.perspective-> Creates the joint controller gui framework
    ├── cv_ur5_project.rviz-> Rviz default layout with gazebo cam
    ├── launch
    │   ├── controller_utils.launch-> Starts the robot_state_publisher
    │   ├── ur5_cv_control.launch-> Main launch file for ur5 with gazebo simulation, block, table, and Rviz
    │   ├── ur5.launch-> Sublaunch file for the ur5 robot itself
    │   ├── ur5_robotiq.launch-> Stand alone launch example for just pulling up the ur5 in Rviz
    │   ├── ur5_upload.launch-> Pushes the ur5 robot to the ROS server
    │   └── viewer.launch-> Opens the Rviz viewer
    ├── meshes
    │   ├── robotiq_adapter.dae-> Gripper 3D model
    │   ├── table_link.stl-> Table 3D model
    │   └── WhiteBox.stl-> Table box 3D model
    ├── package.xml-> Relevant ros packages for gazebo, trajectory, rviz visualization, message, etc..
    ├── src
    │   ├── basic_shapes.cpp-> Debugging script that drops in shapes in an environment
    │   ├── cv_joint_pub.py->publishes a predefined array of 6 joint values for the ur5. Requires ur5_controller.py to be running.
    │   ├── delayed_joint_state_publisher.py-> Debugging script for observing ur5 joint states. Waits an iteration before reading
    │   ├── kinematic_control_ur5.py-> Script run to control ur5 via mediapipe published messages. Requires ur5_controller.py to be running.
    │   ├── marker_subscriber-> Debugging script that subscribes to the "visualization_marker" topic to listen for Marker messages
    │   ├── model_state_tf_publisher.py-> listens to Gazebo's /gazebo/model_states topic and publishes static TF frames
    │   ├── roboticsgroup_gazebo_plugins->Has the ROBOTICSGROUP_GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN for gripper action.
    │   ├── ur5_controller.py-> Moves the robot, listens to joint states and activates the controllers
    │   ├── ur5_FKandJacob.py-> Calculate the forward kinematics from the base_link of the ur5 to the tool0 frame FK, J, and IK for ur5
    │   └── ur5_urdf_parser.py-> constructs a symbolic forward kinematics model of the UR5 robot using its URDF
    ├── urdf->Various URDFS and robot part models, camera, table, block, etc. use in the launches
    │   ├── adapter.urdf
    │   ├── adapter.urdf.xacro
    │   ├── adapter.xacro
    │   ├── common.gazebo.xacro
    │   ├── cube_pick_place.urdf
    │   ├── materials.xacro
    │   ├── MockGripper.xacro
    │   ├── Table.urdf
    │   ├── Table.xacro
    │   ├── ur5_joint_limited_robot.urdf.xacro
    │   ├── ur5_robot.urdf
    │   ├── ur5_robot.urdf.xacro
    │   ├── ur5.urdf.xacro
    │   ├── ur.gazebo.xacro
    │   ├── ur.transmission.xacro
    │   └── viewer_cam.urdf
    └── worlds
        └── empty.world -> Empty gazebo world for physic simulation
