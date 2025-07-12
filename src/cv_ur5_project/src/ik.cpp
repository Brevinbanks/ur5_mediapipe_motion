#include <ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <kdl/chain.hpp>
#include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace KDL;
// TODO: set move_to_target to false for retrieving current xyzrpy without moving the robot; or set move_to_target to true for executing desired motion. Please compile after making changes 
bool move_to_target = true;


// TODO: copy and paste your own KDL chain
KDL::Chain LWR() {
      
      KDL::Chain chain;

      // base -> shoulder
      KDL::Frame R, T;
      T = KDL::Frame(KDL::Vector(0, 0, 0.08937860720288403));
      R = KDL::Frame(KDL::Rotation::RPY(0,0,3.786164542560357e-05));
      KDL::Frame frame1 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frame1));
     
      // shoulder -> upper_arm
      T = KDL::Frame(KDL::Vector(0.0002140505392814961, 0, 0));
      R = KDL::Frame(KDL::Rotation::RPY(1.5707670301715,0,-4.581529412304997e-05));
      KDL::Frame frame2 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame2));


      // upper_arm -> forearm
      T = KDL::Frame(KDL::Vector(-0.4253459842588479, 0, 0));
      R = KDL::Frame(KDL::Rotation::RPY(3.141110703843447,3.140795530350098,-3.141554994853042));
      KDL::Frame frame3 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame3));

      // forearm -> wrist 1
      T = KDL::Frame(KDL::Vector(-0.3925179488687551, 0.0007403918978620855, 0.1103796457395058));
      R = KDL::Frame(KDL::Rotation::RPY(3.13488506915209,3.141575598062244,-3.141582825974094));
      KDL::Frame frame4 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame4));

      // wrist 1 -> wrist 2
      T = KDL::Frame(KDL::Vector(-0.0003380869655392727, -0.09484577543107511, 5.168771128911171e-05));
      R = KDL::Frame(KDL::Rotation::RPY(1.570251360970775,0,3.602818805379087e-06));
      KDL::Frame frame5 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame5));

      // wrist 2 -> wrist 3
      T = KDL::Frame(KDL::Vector(0.00029033673048012,0.08253903740319936,5.245445469226094e-05));
      R = KDL::Frame(KDL::Rotation::RPY(1.57143183756993,3.141592653589793,-3.141535372924156));
      KDL::Frame frame6 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame6));

      // wrist 3 -> end_effector 
      T = KDL::Frame(KDL::Vector(0.0, 0.0, 0.0));//0.235))));
      R = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0));
      KDL::Frame frame7 = KDL::Frame(T * R);
      chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame7));
      
      return chain;
}


/*	Callback for reading joint_states values.
*/
bool initialized = false;
bool joint_received = false;
sensor_msgs::JointState joints;
void get_joint_states(const sensor_msgs::JointState & data) {
	if (initialized) {
    joints.position[0] = data.position[2];
    joints.position[1] = data.position[1];
    joints.position[2] = data.position[0];
    joints.position[3] = data.position[3];
    joints.position[4] = data.position[4];
    joints.position[5] = data.position[5];
		joint_received = true;
	} else {
    for (int i = 0; i < 6; i++) {
  		joints.position.push_back(0.0);
    }
	}
	initialized = true;
}


/*	Initialize the joint positions with a non-zero value to be used in the solvers.
*/
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _jointpositions(i) = _init;
}


/*	Initialize a joint command point.
*/
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init) {
      for (int i = 0; i < _nj; ++i)
            _pt.positions.push_back(_init);
}


/* 	bound the joint values to be between -pi to pi 
*/
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj) {
	for (int i = 0; i < _nj; ++i){
		 while(_jointpositions(i) >= M_PI)
				_jointpositions(i) -= 2*M_PI;
		 while(_jointpositions(i) <= -M_PI)
				_jointpositions(i) += 2*M_PI;
		_point.positions[i] = _jointpositions(i);
	}

}

// for lab 4 callback function 
double targ_x, targ_y, targ_z, targ_roll, targ_pitch, targ_yaw = 0.0;

void get_point_real(const geometry_msgs::Twist & _data){
	if(initialized){
		targ_x = _data.linear.x;
		targ_y = _data.linear.y;
		targ_z = _data.linear.z;
		targ_roll = _data.angular.x;
		targ_pitch = _data.angular.y;
		targ_yaw = _data.angular.z;
	}
	
}



int main(int argc, char * argv[]) {
	// Define the kinematic chain related.
	KDL::Chain chain = LWR();
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);
	KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6);
	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = KDL::JntArray(nj);
	initialize_joints(jointpositions, nj, 0.2);
	geometry_msgs::Twist xyz;
	KDL::Frame cartpos;

	// Define the ros node related.
  	ros::init(argc, argv, "ik");
  	ros::NodeHandle nh_;
  	int loop_freq = 10;
  	float dt = (float) 1 / loop_freq;
  	ros::Rate loop_rate(loop_freq);
	tf::TransformBroadcaster br;
	tf::Transform tool_in_base_link;
  	ros::Publisher cmd_pub = nh_.advertise < trajectory_msgs::JointTrajectory > ("/scaled_pos_joint_traj_controller/command", 10);
	ros::Subscriber jointStates_sub = nh_.subscribe("/joint_states", 10, get_joint_states);
	ros::Publisher xyzrpy_pub = nh_.advertise < geometry_msgs::Twist > ("/robot/worldpos", 10);
	// TODO: Define subscriber point_real_sub to subscribe from the subscriber you added in move_with_homography.cpp
	// The callback function get_point_real is already created
	ros::Subscriber point_real_sub = nh_.subscribe("/ik_point_real", 10, get_point_real);
	
  	// Define trajectory point.
  	trajectory_msgs::JointTrajectoryPoint pt;
  	initialize_points(pt, nj, 0.0);


	trajectory_msgs::JointTrajectory joint_cmd;
	
  	joint_cmd.joint_names.push_back("shoulder_pan_joint");
  	joint_cmd.joint_names.push_back("shoulder_lift_joint");
  	joint_cmd.joint_names.push_back("elbow_joint");
  	joint_cmd.joint_names.push_back("wrist_1_joint");
  	joint_cmd.joint_names.push_back("wrist_2_joint");
  	joint_cmd.joint_names.push_back("wrist_3_joint");

	while(!joint_received){
		ros::spinOnce();
		loop_rate.sleep();
	}

	jointpositions(0) = joints.position[0];
	jointpositions(1) = joints.position[1];
	jointpositions(2) = joints.position[2];
	jointpositions(3) = joints.position[3];
	jointpositions(4) = joints.position[4];
	jointpositions(5) = joints.position[5];


	KDL::Frame target_pt;
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	target_pt.p[0] = targ_x;
	target_pt.p[1] = targ_y;
	target_pt.p[2] = targ_z;
	target_pt.M = KDL::Rotation::RPY(targ_roll, targ_pitch, targ_yaw);
	int ret = iksolver.CartToJnt(jointpositions, target_pt, jointpositions_new);

	eval_points(pt, jointpositions_new, nj);
	pt.time_from_start = ros::Duration(5.0);
	joint_cmd.header.stamp = ros::Time::now();
	joint_cmd.points.push_back(pt);

  while (ros::ok()) {
		if (initialized) {
			bool kinematics_status;
			double roll, pitch, yaw, x, y, z;
			for (int i = 0; i < 6; i++) {
				jointpositions(i) = joints.position[i];
			}
			// Debugging code. echo "/robot/worldpos" in a terminal or watch "tool_from_kdl" in rviz. 
			kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
			if (kinematics_status >= 0) {
				xyz.linear.x = cartpos.p[0];
				xyz.linear.y = cartpos.p[1];
				xyz.linear.z = cartpos.p[2];
				cartpos.M.GetRPY(roll, pitch, yaw);
				xyz.angular.x = roll;
				xyz.angular.y = pitch;
				xyz.angular.z = yaw;
				tool_in_base_link.setOrigin( tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]) );
				tf::Quaternion tool_orientation;
				tool_orientation.setRPY(roll, pitch,  yaw); 
				tool_in_base_link.setRotation( tool_orientation );
				br.sendTransform(tf::StampedTransform(tool_in_base_link, ros::Time::now(), "base", "tool_from_kdl"));
			}
			joint_cmd.header.stamp = ros::Time::now();
			if (move_to_target) {
				cmd_pub.publish(joint_cmd);
			}
			xyzrpy_pub.publish(xyz);
		}
		loop_rate.sleep();
		ros::spinOnce();     
	} 
	return 0;
}

