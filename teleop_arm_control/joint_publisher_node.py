#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
import math
import numpy as np
from transforms3d.euler import quat2euler, euler2quat

class JointPublisherNode(Node):
    """
    A node to receive a PoseArray (up to 2 poses for 2 arms) and
    publish corresponding joint angles for 6-DOF arms. Also listens to two separate
    gripper command topics for each arm.
    """

    def __init__(self):
        super().__init__('joint_publisher_node')
        
        # Joint names for 2 arms (6 DOFs each), plus grippers:
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 
            'finger1_joint', 'finger2_joint',
            'joint1_second_arm', 'joint2_second_arm', 'joint3_second_arm',
            'joint4_second_arm', 'joint5_second_arm', 'joint6_second_arm', 
            'finger1_joint_second_arm', 'finger2_joint_second_arm'
        ]
        
        # Current joint positions for both arms (now 16 total joints)
        self.joint_positions = [0.0] * 16
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribe to multi-poses
        self.poses_sub = self.create_subscription(
            PoseArray,
            '/teleop/target_poses',
            self.target_poses_callback,
            10
        )
        
        # Separate subscriptions for each arm's gripper:
        self.gripper_sub_arm1 = self.create_subscription(
            Bool,
            '/teleop/gripper_command_arm1',
            self.gripper_command_arm1_callback,
            10
        )
        self.gripper_sub_arm2 = self.create_subscription(
            Bool,
            '/teleop/gripper_command_arm2',
            self.gripper_command_arm2_callback,
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info('Joint publisher node initialized: controlling 2 6-DOF robot arms')

    def target_poses_callback(self, msg: PoseArray):
        """
        Handle an array of target poses. For each pose in msg.poses,
        do a simple IK to set the corresponding subset of joints.
        """
        # Pose[0] -> first arm, Pose[1] -> second arm
        for i, pose in enumerate(msg.poses):
            if i > 1:
                # If you only have 2 arms, ignore any extra
                break

            # Basic XYZ "IK" with 6-DOF wrist
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            offset = i * 8  # each "arm" now has 8 relevant joints (6 arm + 2 gripper)

            # Base rotation
            self.joint_positions[offset + 0] = math.atan2(y, x)

            xy_dist = math.sqrt(x*x + y*y)
            # Simple approach for shoulder & elbow
            self.joint_positions[offset + 1] = math.atan2(z, xy_dist) - math.pi/6
            self.joint_positions[offset + 2] = math.atan2(z, xy_dist)/2

            # Extract orientation from quaternion (now handling 3 wrist joints)
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            # Convert quaternion to euler angles
            roll, pitch, yaw = quat2euler([qw, qx, qy, qz], 'rxyz')

            # Map to "wrist" angles - now with 3 joints for full orientation control
            self.joint_positions[offset + 3] = roll  # Wrist roll (joint4)
            self.joint_positions[offset + 4] = pitch  # Wrist pitch (joint5)
            self.joint_positions[offset + 5] = yaw  # Wrist yaw (joint6)

            self.get_logger().debug(
                f'Arm {i+1}: base={self.joint_positions[offset]:.2f}, '
                f'shoulder={self.joint_positions[offset+1]:.2f}, '
                f'elbow={self.joint_positions[offset+2]:.2f}, '
                f'wrist_roll={self.joint_positions[offset+3]:.2f}, '
                f'wrist_pitch={self.joint_positions[offset+4]:.2f}, '
                f'wrist_yaw={self.joint_positions[offset+5]:.2f}'
            )

    def gripper_command_arm1_callback(self, msg: Bool):
        """
        If True => open, else => close for the first arm's gripper
        """
        if msg.data:
            # Open gripper
            self.joint_positions[6] = 0.04
            self.joint_positions[7] = 0.04
            self.get_logger().debug('Arm 1: Opening gripper')
        else:
            # Close gripper
            self.joint_positions[6] = 0.0
            self.joint_positions[7] = 0.0
            self.get_logger().debug('Arm 1: Closing gripper')

    def gripper_command_arm2_callback(self, msg: Bool):
        """
        If True => open, else => close for the second arm's gripper
        """
        if msg.data:
            # Open gripper
            self.joint_positions[14] = 0.04
            self.joint_positions[15] = 0.04
            self.get_logger().debug('Arm 2: Opening gripper')
        else:
            # Close gripper
            self.joint_positions[14] = 0.0
            self.joint_positions[15] = 0.0
            self.get_logger().debug('Arm 2: Closing gripper')

    def publish_joint_states(self):
        """Publish the current joint states on /joint_states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()