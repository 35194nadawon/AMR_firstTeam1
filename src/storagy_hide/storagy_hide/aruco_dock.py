#!/usr/bin/env python3
"""R2 — ArUco 정밀 도킹 + hideout 복귀 Nav2.

담당(R2):
  - RETURN: hideout 좌표로 Nav2 navigate_to_pose → /hide/return_arrived
  - DOCK: 카메라 ArUco P제어 → /hide/dock_done
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except Exception:
    _HAS_CV = False


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ArucoDock(Node):
    def __init__(self):
        super().__init__('hide_aruco_dock')

        self.declare_parameter('target_id', 0)
        self.declare_parameter('stop_distance_m', 0.4)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('hideout_x', -4.55)
        self.declare_parameter('hideout_y', -2.2)
        self.declare_parameter('hideout_yaw', 0.0)

        self.target_id = int(self.get_parameter('target_id').value)
        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.hideout_x = float(self.get_parameter('hideout_x').value)
        self.hideout_y = float(self.get_parameter('hideout_y').value)
        self.hideout_yaw = float(self.get_parameter('hideout_yaw').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/hide/dock_done', 10)
        self.return_arrived_pub = self.create_publisher(Bool, '/hide/return_arrived', 10)
        self.event_pub = self.create_publisher(String, '/robot_events', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/dashboard/nav_goal', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        hide_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Image, '/camera/color/image_raw', self._on_image, 1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_info, 1)
        self.create_subscription(String, '/hide/state', self._on_state, hide_state_qos)

        self.state = 'FREEZE'
        self.camera_info = None
        self.docked = False
        self.return_nav_active = False
        self.return_nav_done = False
        self.return_retry_count = 0
        self.return_retry_timer = None
        self.goal_handle = None
        self.bridge = CvBridge() if _HAS_CV else None
        if _HAS_CV:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.get_logger().info(
            f'R2 ArucoDock 시작 (target_id={self.target_id}, stop={self.stop_distance}m, '
            f'hideout=({self.hideout_x:.2f}, {self.hideout_y:.2f}), cv2={_HAS_CV})')

    def _hideout_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.hideout_x
        pose.pose.position.y = self.hideout_y
        pose.pose.orientation = quaternion_from_yaw(self.hideout_yaw)
        return pose

    def _publish_event(self, text: str):
        self.event_pub.publish(String(data=text))

    def _publish_return_goal_marker(self):
        msg = PoseStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.hideout_x
        msg.pose.position.y = self.hideout_y
        msg.pose.orientation = quaternion_from_yaw(self.hideout_yaw)
        self.nav_goal_pub.publish(msg)

    def _clear_return_retry_timer(self):
        if self.return_retry_timer is not None:
            self.return_retry_timer.cancel()
            self.return_retry_timer = None

    def _schedule_return_retry(self):
        if self.state != 'RETURN' or self.return_nav_done:
            return
        if self.return_retry_count >= 5:
            self.get_logger().error('Hideout return Nav2 failed after retries')
            self._publish_event('복귀 Nav2 실패 — hideout 목표 거부/타임아웃')
            return
        self.return_retry_count += 1
        self._clear_return_retry_timer()
        self.return_retry_timer = self.create_timer(1.5, self._retry_return_nav_once)

    def _retry_return_nav_once(self):
        self._clear_return_retry_timer()
        if self.state != 'RETURN' or self.return_nav_done:
            return
        self.return_nav_active = False
        self._start_return_nav()

    def _on_state(self, msg: String):
        new_state = msg.data.strip()
        if new_state == self.state:
            return

        prev = self.state
        self.state = new_state
        self.docked = False
        self._tag_visible = False

        if prev == 'RETURN' and new_state != 'RETURN':
            self._cancel_return_nav()

        if new_state == 'RETURN':
            self.return_nav_done = False
            self.return_retry_count = 0
            self._clear_return_retry_timer()
            # LLM Nav2 cancel 직후 Nav2가 goal을 받을 수 있도록 짧게 대기
            self.return_retry_timer = self.create_timer(0.8, self._retry_return_nav_once)
        elif new_state != 'DOCK':
            self.return_nav_active = False
            self.return_nav_done = False

    def _start_return_nav(self):
        if self.state != 'RETURN':
            return
        if self.return_nav_active or self.return_nav_done:
            return
        if not self.nav_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('Nav2 navigate_to_pose unavailable for hideout return')
            self._schedule_return_retry()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._hideout_pose()
        self.get_logger().info(
            'RETURN: Nav2 hideout goal '
            f'x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f}')
        self._publish_event(
            f"복귀 Nav2 시작 — hideout(x={self.hideout_x:.2f}, y={self.hideout_y:.2f})")
        self._publish_return_goal_marker()
        self.return_nav_active = True
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._return_goal_response)

    def _return_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Hideout return goal rejected by Nav2')
            self.return_nav_active = False
            self._publish_event('복귀 Nav2 목표 거부 — 재시도')
            self._schedule_return_retry()
            return
        self.get_logger().info('Hideout return goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._return_result)

    def _return_result(self, future):
        self.return_nav_active = False
        self.goal_handle = None
        if self.state != 'RETURN':
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.return_nav_done = True
            self.return_retry_count = 0
            self.return_arrived_pub.publish(Bool(data=True))
            self.get_logger().info('RETURN: arrived at hideout — /hide/return_arrived')
            self._publish_event('hideout 도착 (/hide/return_arrived)')
            return

        self.get_logger().warn(f'Hideout return failed (status={status})')
        self._publish_event(f'복귀 Nav2 실패 (status={status}) — 재시도')
        self._schedule_return_retry()

    def _cancel_return_nav(self):
        if self.goal_handle is None:
            self.return_nav_active = False
            return
        self.get_logger().info('Cancelling hideout return Nav2 goal')
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.return_nav_active = False

    def _on_info(self, msg: CameraInfo):
        self.camera_info = msg
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image):
        if self.state != 'DOCK':
            return

        if self.docked:
            self.cmd_pub.publish(Twist())
            return

        if not _HAS_CV or self.camera_info is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            corners, ids, _ = detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if ids is None or self.target_id not in ids.flatten():
            self._tag_visible = False
            self.cmd_pub.publish(Twist())
            return

        if not getattr(self, '_tag_visible', False):
            self.get_logger().info(f'ArUco tag detected: id={self.target_id}')
        self._tag_visible = True

        i = list(ids.flatten()).index(self.target_id)
        marker_len = 0.15
        objp = np.array([
            [-marker_len / 2,  marker_len / 2, 0.0],
            [ marker_len / 2,  marker_len / 2, 0.0],
            [ marker_len / 2, -marker_len / 2, 0.0],
            [-marker_len / 2, -marker_len / 2, 0.0],
        ], dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            objp, corners[i][0], self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            self.get_logger().warning('ArUco pose estimation failed')
            return

        x = float(tvec[0][0])
        z = float(tvec[2][0])
        cmd = Twist()

        if z <= self.stop_distance + 0.03 and abs(x) < 0.045:
            self.cmd_pub.publish(cmd)
            self.docked = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info(f'ArUco 도킹 완료: z={z:.2f}m, x={x:+.3f}m')
            self._publish_event('도킹 완료 (/hide/dock_done)')
            return

        cmd.angular.z = float(np.clip(-1.8 * x, -0.55, 0.55))
        if abs(x) < 0.15 and z > self.stop_distance:
            cmd.linear.x = float(np.clip(
                0.35 * (z - self.stop_distance), 0.03, 0.10))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
