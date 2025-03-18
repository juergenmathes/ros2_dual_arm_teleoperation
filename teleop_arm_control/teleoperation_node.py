#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
import math

class TeleoperationNode(Node):
    """
    Node that tracks up to two hands using computer vision and publishes:
      - A PoseArray with one Pose for each hand (up to 2)
      - Separate gripper commands for each hand
    """

    def __init__(self):
        super().__init__('teleoperation_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Subscribe to camera images
        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10
        )
        
        # Publisher for up to two poses in a PoseArray
        self.poses_pub = self.create_publisher(
            PoseArray,
            '/teleop/target_poses',
            10
        )

        # Two separate gripper publishers
        self.gripper_pub_arm1 = self.create_publisher(
            Bool,
            '/teleop/gripper_command_arm1',
            10
        )
        self.gripper_pub_arm2 = self.create_publisher(
            Bool,
            '/teleop/gripper_command_arm2',
            10
        )
        
        # Workspace mapping
        self.x_min, self.x_max = 0.2, 0.6
        self.y_min, self.y_max = -0.3, 0.3
        self.z_value = 0.3
        
        # Last poses for smoothing
        self.last_pose_1 = None
        self.last_pose_2 = None

        # Track hand closure state
        self.last_hand_closed_1 = False
        self.last_hand_closed_2 = False

        self.smoothing_factor = 0.5
        
        # Debug image publisher
        self.debug_img_pub = self.create_publisher(
            Image,
            '/teleop/debug_image',
            10
        )

        self.get_logger().info('Teleoperation node initialized: tracking up to 2 hands')

    def camera_callback(self, msg):
        """Process incoming camera frames and detect up to two hands."""
        try:
            # Convert ROS Image message to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image with MediaPipe
            results = self.process_image(cv_image)
            
            # Prepare a PoseArray for up to two hands
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = self.get_clock().now().to_msg()
            pose_array_msg.header.frame_id = "base_link"

            # Potential commands for each hand
            gripper_command_1 = None
            gripper_command_2 = None

            # If hands detected, process them
            if results.multi_hand_landmarks:
                # Up to 2 hands
                num_hands = min(len(results.multi_hand_landmarks), 2)

                for i in range(num_hands):
                    landmarks = results.multi_hand_landmarks[i]
                    
                    if i == 0:
                        pose_1, gripper_command_1 = self.calculate_commands(
                            landmarks, cv_image.shape, previous_pose=self.last_pose_1
                        )
                        self.last_pose_1 = pose_1
                        pose_array_msg.poses.append(pose_1)
                    elif i == 1:
                        pose_2, gripper_command_2 = self.calculate_commands(
                            landmarks, cv_image.shape, previous_pose=self.last_pose_2
                        )
                        self.last_pose_2 = pose_2
                        pose_array_msg.poses.append(pose_2)

            # Publish PoseArray (could be 0, 1, or 2 poses)
            self.poses_pub.publish(pose_array_msg)

            # Handle separate gripper commands
            # Hand #1 => Arm #1
            if gripper_command_1 is not None:
                bool_msg = Bool()
                bool_msg.data = gripper_command_1
                self.gripper_pub_arm1.publish(bool_msg)
                action = "open" if gripper_command_1 else "close"
                self.get_logger().debug(f'Hand 1 -> Arm 1: Gripper {action}')
                self.last_hand_closed_1 = (not gripper_command_1)

            # Hand #2 => Arm #2
            if gripper_command_2 is not None:
                bool_msg = Bool()
                bool_msg.data = gripper_command_2
                self.gripper_pub_arm2.publish(bool_msg)
                action = "open" if gripper_command_2 else "close"
                self.get_logger().debug(f'Hand 2 -> Arm 2: Gripper {action}')
                self.last_hand_closed_2 = (not gripper_command_2)

            # Draw debug info
            if results.multi_hand_landmarks:
                self.draw_debug_image(cv_image, results.multi_hand_landmarks)

            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_img_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {str(e)}')

    def process_image(self, image):
        """Run MediaPipe Hands on the given BGR image."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        return results

    def calculate_commands(self, hand_landmarks, image_shape, previous_pose=None):
        """
        Calculate target Pose from hand landmarks, plus a gripper command (open/close).
        """
        image_height, image_width, _ = image_shape

        # Index fingertip
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        
        # Map from [0..1] image coords to robot workspace
        x_pos = self.map_value(index_tip.x, 0, 1, self.x_min, self.x_max)
        y_pos = self.map_value(index_tip.y, 0, 1, self.y_max, self.y_min)  # invert y
        z_pos = self.z_value  # fixed in this example

        # Rotation from index & middle finger MCP
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        dx = middle_mcp.x - index_mcp.x
        dy = middle_mcp.y - index_mcp.y
        angle = math.atan2(dy, dx)

        # Create Pose
        pose = Pose()
        pose.position.x = x_pos
        pose.position.y = y_pos
        pose.position.z = z_pos
        
        # z-axis rotation only
        pose.orientation.w = math.cos(angle / 2)
        pose.orientation.z = math.sin(angle / 2)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        
        # Optional smoothing
        if previous_pose is not None:
            pose = self.smooth_pose(pose, previous_pose)

        # Check gripper command (open vs close)
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        dist = math.sqrt(
            (thumb_tip.x - index_tip.x)**2 +
            (thumb_tip.y - index_tip.y)**2 +
            (thumb_tip.z - index_tip.z)**2
        )
        hand_closed = dist < 0.1  # threshold
        # If closed => False (close gripper), else => True (open)
        gripper_cmd = not hand_closed

        return pose, gripper_cmd

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Linear map a value from [in_min..in_max] to [out_min..out_max]."""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def smooth_pose(self, new_pose, old_pose):
        """Simple linear smoothing between old and new poses."""
        alpha = self.smoothing_factor
        smoothed = Pose()

        # Position
        smoothed.position.x = old_pose.position.x*(1-alpha) + new_pose.position.x*alpha
        smoothed.position.y = old_pose.position.y*(1-alpha) + new_pose.position.y*alpha
        smoothed.position.z = old_pose.position.z*(1-alpha) + new_pose.position.z*alpha
        
        # Orientation
        smoothed.orientation.w = old_pose.orientation.w*(1-alpha) + new_pose.orientation.w*alpha
        smoothed.orientation.x = old_pose.orientation.x*(1-alpha) + new_pose.orientation.x*alpha
        smoothed.orientation.y = old_pose.orientation.y*(1-alpha) + new_pose.orientation.y*alpha
        smoothed.orientation.z = old_pose.orientation.z*(1-alpha) + new_pose.orientation.z*alpha
        
        return smoothed
    
    def draw_debug_image(self, image, hand_landmarks_list):
        """Draw landmarks on debug image."""
        for hand_landmarks in hand_landmarks_list:
            self.mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )
        
        cv2.putText(
            image,
            "Open hand: Open gripper",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        cv2.putText(
            image,
            "Closed hand: Close gripper",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()