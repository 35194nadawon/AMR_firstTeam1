#!/usr/bin/env python3
import sys
import os
import unittest
from unittest.mock import patch, MagicMock
import numpy as np

# Adjust sys.path to find storagy_hide
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(workspace_dir, 'src', 'storagy_hide'))

import rclpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

from storagy_hide.aruco_dock import ArucoDock

class TestArucoDock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # We can construct the node
        self.node = ArucoDock()
        self.bridge = CvBridge()
        
        # Mock publishers to record messages sent
        self.node.cmd_pub = MagicMock()
        self.node.done_pub = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_1_aruco_id_0_detection(self):
        """1. ArUco ID 0 검출 테스트"""
        # Set state to DOCK to pass the state gate
        state_msg = String(data='DOCK')
        self.node._on_state(state_msg)

        # Set mock CameraInfo
        info_msg = CameraInfo()
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.node._on_info(info_msg)

        # Draw ArUco ID 0
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker_img = np.zeros((200, 200), dtype=np.uint8)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            cv2.aruco.generateImageMarker(aruco_dict, 0, 200, marker_img, 1)
        else:
            cv2.aruco.drawMarker(aruco_dict, 0, 200, marker_img, 1)

        # Place marker on a 640x480 white canvas
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
        frame[140:340, 220:420, 0] = marker_img
        frame[140:340, 220:420, 1] = marker_img
        frame[140:340, 220:420, 2] = marker_img

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Trigger image callback
        self.node._on_image(img_msg)

        # Verify ID 0 was detected and _tag_visible is True
        self.assertTrue(getattr(self.node, '_tag_visible', False))
        print("\n[P2 TEST 1/3] PASS - ArUco ID 0 detection")

    def test_2_pose_and_p_control(self):
        """2. x=+0.10m, z=1.00m, stop_distance=0.40m에서 P제어 Twist 검증"""
        # Ensure state is DOCK
        state_msg = String(data='DOCK')
        self.node._on_state(state_msg)

        # CameraInfo setup
        info_msg = CameraInfo()
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.node._on_info(info_msg)

        # Setup SolvePnP mocks to return x = +0.10m, z = 1.00m
        mock_rvec = np.zeros((3, 1), dtype=np.float32)
        mock_tvec = np.array([[0.10], [0.0], [1.00]], dtype=np.float32)
        mock_rot = np.eye(3, dtype=np.float32)

        frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        with patch('cv2.solvePnP', return_value=(True, mock_rvec, mock_tvec)), \
             patch('cv2.Rodrigues', return_value=(mock_rot, None)):
            
            mock_corners = [np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=np.float32)]
            mock_ids = np.array([[0]], dtype=np.int32)
            
            if hasattr(cv2.aruco, 'ArucoDetector'):
                with patch('cv2.aruco.ArucoDetector.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)
            else:
                with patch('cv2.aruco.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)

        # Check Twist command published to cmd_pub
        self.node.cmd_pub.publish.assert_called()
        published_twist = self.node.cmd_pub.publish.call_args[0][0]
        
        # Expected Twist:
        # linear.x = clip(0.35 * (z - stop), 0.03, 0.10) => 0.35 * (1.00 - 0.40) = 0.21 => clipped to 0.10
        # angular.z = clip(-1.8 * x, -0.55, 0.55) => -1.8 * 0.10 = -0.18
        self.assertAlmostEqual(published_twist.linear.x, 0.10, places=2)
        self.assertAlmostEqual(published_twist.angular.z, -0.18, places=2)
        print("\n[P2 TEST 2/3] PASS - Pose and P-control command")

    def test_3_dock_done_once_and_state_gate(self):
        """3. 도킹 완료 시 /hide/dock_done=True 1회 발행 및 FREEZE 상태 게이트 검증"""
        # Ensure state is DOCK
        state_msg = String(data='DOCK')
        self.node._on_state(state_msg)

        # CameraInfo setup
        info_msg = CameraInfo()
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.node._on_info(info_msg)

        # A: Verify dock_done=True is published only once
        # Setup mock solvePnP to satisfy dock success condition: z <= stop_distance + 0.03 (0.43m), |x| < 0.045
        # Set x = 0.02, z = 0.41m
        mock_rvec = np.zeros((3, 1), dtype=np.float32)
        mock_tvec = np.array([[0.02], [0.0], [0.41]], dtype=np.float32)
        mock_rot = np.eye(3, dtype=np.float32)

        frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # First image frame triggering completion
        with patch('cv2.solvePnP', return_value=(True, mock_rvec, mock_tvec)), \
             patch('cv2.Rodrigues', return_value=(mock_rot, None)):
            mock_corners = [np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=np.float32)]
            mock_ids = np.array([[0]], dtype=np.int32)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                with patch('cv2.aruco.ArucoDetector.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)
            else:
                with patch('cv2.aruco.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)

        # Second image frame (should not trigger dock_done publisher again as self.docked is True)
        with patch('cv2.solvePnP', return_value=(True, mock_rvec, mock_tvec)), \
             patch('cv2.Rodrigues', return_value=(mock_rot, None)):
            mock_corners = [np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=np.float32)]
            mock_ids = np.array([[0]], dtype=np.int32)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                with patch('cv2.aruco.ArucoDetector.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)
            else:
                with patch('cv2.aruco.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)

        # Verify done_pub.publish was called exactly once
        self.assertEqual(self.node.done_pub.publish.call_count, 1)
        published_done = self.node.done_pub.publish.call_args[0][0]
        self.assertTrue(published_done.data)

        # B: Verify state gate for FREEZE state
        # Reset mocks
        self.node.cmd_pub.reset_mock()
        self.node.done_pub.reset_mock()

        # Change state to FREEZE
        state_msg = String(data='FREEZE')
        self.node._on_state(state_msg)

        # Send image frame
        with patch('cv2.solvePnP', return_value=(True, mock_rvec, mock_tvec)), \
             patch('cv2.Rodrigues', return_value=(mock_rot, None)):
            mock_corners = [np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=np.float32)]
            mock_ids = np.array([[0]], dtype=np.int32)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                with patch('cv2.aruco.ArucoDetector.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)
            else:
                with patch('cv2.aruco.detectMarkers', return_value=(mock_corners, mock_ids, [])):
                    self.node._on_image(img_msg)

        # In FREEZE state, cmd_pub.publish should NOT be called at all
        self.node.cmd_pub.publish.assert_not_called()
        self.node.done_pub.publish.assert_not_called()
        print("\n[P2 TEST 3/3] PASS - Dock done once and state gate")

if __name__ == '__main__':
    # Run tests and print overall summary
    suite = unittest.TestLoader().loadTestsFromTestCase(TestArucoDock)
    runner = unittest.TextTestRunner(verbosity=0)
    result = runner.run(suite)
    
    if result.wasSuccessful():
        print("[P2 SUMMARY] 3/3 PASS")
        sys.exit(0)
    else:
        print("[P2 SUMMARY] FAIL")
        sys.exit(1)
