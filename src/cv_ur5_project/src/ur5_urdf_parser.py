
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tf2_ros
from tf.transformations import quaternion_from_matrix

import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion

from urdf_parser_py.urdf import Robot
import tf.transformations as tf
import numpy as np
import sympy as sp

class ur5_urdf_parser(object):
    def __init__(self,robot = Robot.from_parameter_server(),
                 link_names = ['world', 'base_link', 'shoulder_link', 'upper_arm_link','forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']
                ,joint_names = ['world_joint', 'base_link-base_fixed_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',  'wrist_2_joint', 'wrist_3_joint', 'wrist_3_link-tool0_fixed_joint']
                ):
        # print(robot)
        self.robot = robot
        self.link_names = link_names
        self.joint_names = joint_names
        self.symbolic_var_counter = 0
        self.robot_tree = self.build_robot()
        self.frame_stack = []
        self.create_symbolicFK()
        
        # base_link = robot.get_root()
        # link_names = robot.link_map.keys()#[len(robot.link_map)-1]
        # joint_names = robot.joint_map.keys()#[len(robot.link_map)-1]
        # joint_names = ['world_joint', 'base_link-base_fixed_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',  'wrist_2_joint', 'wrist_3_joint', 'wrist_3_link-tool0_fixed_joint']
        
    def build_robot(self):
        robot_tree = []
        for link in self.robot.links:
            L_parent = []
            L_child = []
            childxyz = []
            childrpy = []
            childtype = []
            childaxis = []
            if link.name in self.link_names:
                for joint in self.robot.joints:
                    if link.name == joint.child:
                        L_parent.append(joint.name)
                    if link.name == joint.parent:
                        L_child.append(joint.name)
                        childxyz.append(joint.origin.xyz)
                        childrpy.append(joint.origin.rpy)
                        childtype.append(joint.type)
                        childaxis.append(joint.axis)
                robot_tree.append([link.name,L_parent,L_child,childxyz,childrpy,childtype,childaxis])
        robot_tree = self.order_tree(robot_tree)
        return robot_tree

    def order_tree(self,robot_tree):
        tree_details =[]
        root_joint = ""
        k = 0
        for link in robot_tree:
            if link[1]==[]: #has no children
                root_joint = link[2]
                tree_details.append(link)
                robot_tree.pop(k)
                break
            k+=1
        link_searching = True
        next_joint = root_joint
        num_link = len(robot_tree)
        while(link_searching):
            j = 0
            for link in robot_tree:
                for k in range(len(next_joint)):
                    if next_joint[k] in link[1]:

                        link[1] =next_joint[k]
                        link[3] = link[3][k]
                        link[4] = link[4][k]
                        link[5] = link[5][k]
                        link[6] = link[6][k]

                        tree_details.append(link)
                        next_joint = link[2]
                        break
                    elif link[2]==[] and j==num_link-1:
                        tree_details.append(link)
                        link_searching=False
                        break
                j+=1
        j=0
        for joint in tree_details:
            for joint_namer in joint[2]: 
                if joint_namer in self.joint_names:
                    tree_details[j][2] = joint_namer
                    
            j+=1
        
        j = 0
        for joint in tree_details:
            i = 0
            for item in joint:
                tree_details[j][i] = self.extract_single_element(item)
                i+=1
            j+=1
        return tree_details
            
    def extract_single_element(self,item):
        if isinstance(item, list) and all(isinstance(items, str) for items in item):  # Check if item is a list
            if len(item) == 1:  # Check if the list has only one element
                return item[0]  # Return the single element
            # else:
            #     return None  # Return None if the list has more than one element
        else:
            return item  # Return the item itself if it's not a list



    def generate_rotation_matrix_with_sym(self, arr):
        q = sp.symbols('q{}'.format(self.symbolic_var_counter))
        self.symbolic_var_counter += 1

        if arr[0] == 1:
            rotation_matrix = sp.Matrix([[1, 0, 0, 0],
                                    [0, sp.cos(q), -sp.sin(q), 0],
                                    [0, sp.sin(q), sp.cos(q), 0],
                                    [0, 0, 0, 1]])
        elif arr[1] == 1:
            rotation_matrix = sp.Matrix([[sp.cos(q), 0, sp.sin(q), 0],
                                    [0, 1, 0, 0],
                                    [-sp.sin(q), 0, sp.cos(q), 0],
                                    [0, 0, 0, 1]])
        elif arr[2] == 1:
            rotation_matrix = sp.Matrix([[sp.cos(q), -sp.sin(q), 0, 0],
                                    [sp.sin(q), sp.cos(q), 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        else:
            raise ValueError("No value in the array is equal to 1")

        return rotation_matrix

    def generate_prismatic_matrix_with_sym(self, arr):
        q = sp.symbols('q{}'.format(self.symbolic_var_counter))
        self.symbolic_var_counter += 1

        if arr[0] == 1:
            translation_matrix = sp.Matrix([[1, 0, 0, q],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        elif arr[1] == 1:
            translation_matrix = sp.Matrix([[1, 0, 0, 0],
                                    [0, 1, 0, q],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        elif arr[2] == 1:
            translation_matrix = sp.Matrix([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, q],
                                    [0, 0, 0, 1]])
        else:
            raise ValueError("No value in the array is equal to 1")

        return translation_matrix
    


    def create_symbolicFK(self):
        num_sym = 0
        tf_s = []
        for joint in self.robot_tree:
            # print(joint)
            if joint[5]=='revolute' or joint[5]=='prismatic':
                num_sym+=1
        for joint in self.robot_tree:
            if joint[5]!=None:
                rpy_ang = np.array(joint[4]).flatten('C')
                transform_matrix = tf.euler_matrix(rpy_ang[0],rpy_ang[1],rpy_ang[2]) # Extract the 3x3 rotation matrix frs transformation matrix
                xyz_pos = np.array(joint[3]).flatten('C')
                transform_matrix[0,3] = xyz_pos[0]
                transform_matrix[1,3] = xyz_pos[1]
                transform_matrix[2,3] = xyz_pos[2]
                if joint[5] =='revolute' and joint[6]!=None:
                    tfq = self.generate_rotation_matrix_with_sym(np.array(joint[6]).flatten('C'))    
                    tf_s.append(transform_matrix*tfq)
                elif joint[5] =='primsatic' and joint[6]!=None:
                    tfq = self.generate_prismatic_matrix_with_sym(np.array(joint[6]).flatten('C'))    
                    tf_s.append(transform_matrix*tfq)
                else:
                    tf_s.append(transform_matrix)
        self.frame_stack = tf_s

        # for frame in tf_s:
        #     sp.pprint(frame)

    def FK_solve_sym(self,link1,link2):
        
        solved_transform = np.eye(4)
        if link1 not in self.link_names and link2 not in self.link_names:
            raise ValueError("links given to Fk_solve were not found in the given link names. Check the spelling or robot given to urdf parser")
        else:
            start_link = self.link_names.index(link1)
            end_link = self.link_names.index(link2)
            for link_num in range(start_link, end_link):
                if isinstance(self.frame_stack[link_num], sp.Matrix) or isinstance(solved_transform, sp.Matrix):
                    solved_transform = solved_transform*self.frame_stack[link_num]
                else:
                    solved_transform = np.dot(solved_transform,self.frame_stack[link_num])
        if isinstance(solved_transform, sp.Matrix):
            solved_transform = sp.simplify(solved_transform)
        return solved_transform

joint_positions = []
def ur5_joint_sub_callback(msg):
    global joint_positions
    joint_positions = msg.position

if __name__ == "__main__":
 

    parser = ur5_urdf_parser()
    # expression = parser.FK_solve_sym('world','tool0')
 
    rospy.init_node('kinematics_example')

    rospy.Subscriber('joint_states', JointState, ur5_joint_sub_callback)
    rospy.sleep(0.1)
    import ur5_FKandJacob as ur5FK

    while not rospy.is_shutdown():
        
        try:
            q = np.array([joint_positions[3],joint_positions[2],joint_positions[1],joint_positions[4],joint_positions[5],joint_positions[6]])
                
            pose = ur5FK.ForwardKinematics(q)
            J = ur5FK.Jacobian(q)
            # print(pose)
            
            tf_broadcaster = tf2_ros.TransformBroadcaster()

            transform = geometry_msgs.msg.TransformStamped()


            transform.header.frame_id = "base_link"  # Assuming models are defined in the world frame
            transform.child_frame_id = "parser_tool"
    
            transform.header.stamp = rospy.Time.now()
            # Assign translation
            
            transform.transform.translation.x = pose[0, 3]
            transform.transform.translation.y = pose[1, 3]
            transform.transform.translation.z = pose[2, 3]

            # Convert rotation matrix to quaternion
            quaternion = quaternion_from_matrix(pose)
            # Create a Quaternion message and assign values
            transform.transform.rotation = Quaternion(*quaternion)

            # Publish the transform
            tf_broadcaster.sendTransform(transform)
        except:
            print('not working')
    
        rospy.sleep(0.1)