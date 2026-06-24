#!/usr/bin/env python3
"""R2 — ArUco 정밀 도킹 + 복귀 주행.

담당(R2):
  - 월드 1206_2.sdf 구석에 aruco_0 대기석 배치, points.yaml 에 hideout 좌표 추가
  - /camera/color/image_raw + camera_info 로 cv2.aruco 태그 pose 추정
  - 정렬+접근 P제어로 /cmd_vel 발행, 거리 임계값 도달 시 /hide/dock_done(True) 발행
  - 복귀(RETURN): hideout 좌표로 Nav2 navigate_to_pose 후 도킹 시작
  - (HW) 엔코더 기반 정밀 정지로 교체 가능

상태 게이트: R1 FSM 이 RETURN/DOCK 일 때만 /cmd_vel 을 발행하도록 연동할 것.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except Exception:  # 빌드/검출 라이브러리 없을 때도 노드는 떠야 함
    _HAS_CV = False


class ArucoDock(Node):
    def __init__(self):
        super().__init__('hide_aruco_dock')

        self.declare_parameter('target_id', 0)
        self.declare_parameter('stop_distance_m', 0.4)
        self.target_id = self.get_parameter('target_id').value
        self.stop_distance = self.get_parameter('stop_distance_m').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/hide/dock_done', 10)

        self.create_subscription(Image, '/camera/color/image_raw', self._on_image, 1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_info, 1)
        self.create_subscription(String, '/hide/state', self._on_state, 10)

        self.state = 'FREEZE'
        self.camera_info = None
        self.bridge = CvBridge() if _HAS_CV else None
        if _HAS_CV:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.get_logger().info(
            f'R2 ArucoDock 시작 (target_id={self.target_id}, '
            f'stop={self.stop_distance}m, cv2={_HAS_CV})')

    def _on_state(self, msg: String):
        self.state = msg.data

    def _on_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_image(self, msg: Image):
        # RETURN/DOCK 일 때만 도킹 제어 (제어권 충돌 방지)
        if self.state not in ('RETURN', 'DOCK'):
            return
        if not _HAS_CV or self.camera_info is None:
            return
        # TODO(R2): cv2.aruco.detectMarkers → estimatePoseSingleMarkers 로 태그 상대 pose
        # TODO(R2): yaw/lateral 오차 P제어 → Twist, 거리<stop_distance 면 도킹 완료
        # 예시 골격:
        #   frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #   corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)
        #   ... 정렬/접근 ...
        #   self.cmd_pub.publish(twist)
        #   if reached: self.done_pub.publish(Bool(data=True))
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
