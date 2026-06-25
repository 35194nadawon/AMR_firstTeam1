#!/usr/bin/env python3
"""P2 — ArUco 정밀 도킹 + 복귀 (대기석 은폐) [스켈레톤].

토이 가이드(컨셉 3): 임무 완료 후 RETURN 으로 대기석에 접근하면, 구석/책상 다리의
ArUco 태그를 인식해 오차 1cm 미만으로 정밀 밀착(완전 은폐)하고 도킹 완료를 FSM 에 통지한다.

이 파일은 배선만 되어 있고, 핵심 로직은 TODO 다. 직접 채운다.
구현 가이드: 플랜 문서 "숨는팀 P2 ArUco 도킹" §6 (코드 가이드) 참고.

토픽 계약:
  - 구독: /camera/color/image_raw(Image), /camera/color/camera_info(CameraInfo), /hide/state(String)
  - 발행: /cmd_vel(Twist), /hide/dock_done(Bool)
  - 게이트: /hide/state 가 RETURN/DOCK 일 때만 /cmd_vel 발행
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
except Exception:
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
        self.get_logger().info(f'P2 ArucoDock 시작 (스켈레톤, cv2={_HAS_CV})')

    def _on_state(self, msg: String):
        self.state = msg.data

    def _on_info(self, msg: CameraInfo):
        # TODO(P2): camera_info 에서 K(내부행렬)/D(왜곡) 저장. 가이드 §6-2
        self.camera_info = msg

    def _on_image(self, msg: Image):
        # TODO(P2): 태그 검출 + pose 추정 + P제어 도킹 + /hide/dock_done. 가이드 §6-3
        # 게이트: self.state 가 'RETURN'/'DOCK' 일 때만 /cmd_vel 발행
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
