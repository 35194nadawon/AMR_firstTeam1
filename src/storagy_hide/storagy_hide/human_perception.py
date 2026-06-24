#!/usr/bin/env python3
"""R3 — 사람 감지 + 120도 keepout 영역 산출.

담당(R3):
  - YOLO(사람) + depth(/camera/depth/image_raw)로 사람 거리/방향 추정
  - 사람 pose 배열을 /hide/persons (PoseArray) 로 발행
  - 감지된 사람 전방 120도 부채꼴을 진입금지 영역으로 /hide/keepout_zones (PolygonStamped) 발행
    → R4 가 코스트맵에 주입

기존 yolo_detector(/yolo/person_count, /yolo/detected_image)를 재사용하거나,
여기서 직접 YOLO 추론을 돌려도 됨. R4 와 메시지 포맷만 합의하면 독립 개발 가능.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseArray, PolygonStamped


class HumanPerception(Node):
    def __init__(self):
        super().__init__('hide_human_perception')

        self.declare_parameter('fov_deg', 120.0)
        self.declare_parameter('zone_radius_m', 1.0)
        self.fov_deg = self.get_parameter('fov_deg').value
        self.zone_radius = self.get_parameter('zone_radius_m').value

        self.persons_pub = self.create_publisher(PoseArray, '/hide/persons', 10)
        self.zones_pub = self.create_publisher(PolygonStamped, '/hide/keepout_zones', 10)

        self.create_subscription(Int32, '/yolo/person_count', self._on_count, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth, 1)

        self.get_logger().info(
            f'R3 HumanPerception 시작 (fov={self.fov_deg}도, r={self.zone_radius}m)')

    def _on_count(self, msg: Int32):
        # TODO(R3): yolo 박스 + depth 로 사람 위치 추정 후 /hide/persons 발행
        pass

    def _on_depth(self, msg: Image):
        # TODO(R3): YOLO 박스 중심의 depth 로 거리 추정 → 카메라 내부파라미터로 (x,y) 역투영
        # TODO(R3): 사람 위치 기준 전방 fov_deg 부채꼴 폴리곤 생성 → /hide/keepout_zones 발행
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HumanPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
