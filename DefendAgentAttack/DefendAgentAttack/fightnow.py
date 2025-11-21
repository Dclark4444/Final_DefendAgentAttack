import rclpy
import sys
from rclpy.node import Node
import time

import cv2
import cv_bridge
import numpy as np
import os

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from omx_cpp_interface.msg import ArmGripperPosition, ArmJointAngles


class ExecuteOptimal(Node):
    def __init__(self, player):
        super().__init__('execute_optimal_policy')
        # getting shared directory to be able to access q_matrix

         # get ROS_DOMAIN_ID
        ros_domain_id = os.getenv("ROS_DOMAIN_ID")
        try:
            if int(ros_domain_id) < 10:
                ros_domain_id = "0" + str(int(ros_domain_id))
            else:
                ros_domain_id = str(int(ros_domain_id))
        except Exception:
            ros_domain_id = "00"

        self.get_logger().info(f'ROS_DOMAIN_ID: {ros_domain_id}')   


         # Fetch Actions and states (saved files)
         # Joint action --> [Attack, Defender]
         # Attack Agent can Move_R/L (0,1), arm down in direction of move (2)
         # Defend Agent can Turn to Face (0), arm_back_right/left (1,2) 
        action_pth = os.path.join(self.share, 'matrices', 'action.txt')
        self.actions = np.loadtxt(action_pth)


        # Atack agent states are indicies 0-1 and Defend are 2-3 
        # index 0 represents the Attack Agent stance relative to Defense
        # ==> 0 - in front, 1 - to the Right, 2 - to the left 
        # index 1 represens the A Agent arm up (0), or down (1)
        # index 2 represents the D Agent stance relative to Defense
        # ===> forward facing (0), not seeing (1)
        # index 3 represents the D Agent Arm position 
        # ==> home/up (0), arm_down_right (1), arm_down_left (2)
        state_pth = os.path.join(self.share, 'matrices', 'states.txt')
        self.states = np.loadtxt(state_pth)



        matrix_path = os.path.join(self.share, 'matrices', 'RL_matrix.txt')
        self.q_matrix = np.loadtxt(matrix_path) # np array of q_matrix 
        self.get_logger().info(f'Loaded RL_matrix')


        # color bounds of all items in the environment 
        self.color_bounds = {
        "spear":  (np.array([140, 50, 50]), np.array([170, 255, 255])),
        "shield": (np.array([35, 50, 50]),  np.array([85, 255, 255])),
        "balloon":  (np.array([85, 50, 50]), np.array([125, 255, 255]))
        }

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()


        #Topics 
        cmd_vel_topic = f"/tb{ros_domain_id}/cmd_vel"
        compress_image_topic = f'/tb{self.ros_domain_id}/oakd/rgb/preview/image_raw/compressed'
        scan_topic = f'/tb{self.ros_domain_id}/scan'
        arm_grip = f'/tb{ros_domain_id}/target_gripper_position'
        arm_angles = f'/tb{ros_domain_id}/target_joint_angles'


        # publishing
        self.joint_arm_pub = self.create_publisher(ArmJointAngles, arm_angles, 10)
        self.gripper_pub = self.create_publisher(ArmGripperPosition, arm_grip, 10)
        self.movement_publisher =  self.create_publisher(Twist, cmd_vel_topic, 10)


        # subscriptions 
        self.image_sub = self.create_subscription(CompressedImage, 
            compress_image_topic, 
            self.image_callback, #trigger image for robot to look around (movement)
            10
        )

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, 
            self.scan_callback,10)
        
        
        time.sleep(2)


        # Wait until 2 robots are running this file


        self.perform_action()



    def send_join_info():
        None




def main(args=None):
    rclpy.init(args=args)
    player = sys.argv[1:] # Attack (A), Defend (D)
    node = ExecuteOptimal(player)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





