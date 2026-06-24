#!/usr/bin/env python3
"""R4 — 동적 코스트맵 주입 + keepout 인프라.

담당(R4):
  - storagy/param/navigation2/storagy.yaml 에 정적 keepout 연결:
      costmap_filter_info_server + filter_mask_server + KeepoutFilter 레이어
      (기존 map/1206_sim_1_keepout.* 사용)
  - R3 의 /hide/keepout_zones(사람 120도 부채꼴)를 코스트맵에 동적 주입
      (옵션 1: 이 노드가 폴리곤 → 동적 filter_mask OccupancyGrid 갱신 발행)
      (옵션 2: nav2 의 obstacle/keepout 레이어가 직접 받도록 구성)
  - RViz 로 진입금지 영역 시각화

※ 정적 keepout 은 주로 nav2 yaml 설정 작업이고, 이 노드는 "동적" 부분 담당.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid


class DynamicCostmap(Node):
    def __init__(self):
        super().__init__('hide_dynamic_costmap')

        # 동적 keepout 마스크를 별도 토픽으로 내보내 nav2 filter 가 구독하도록 할 수 있음
        self.mask_pub = self.create_publisher(OccupancyGrid, '/hide/dynamic_keepout_mask', 1)
        self.create_subscription(PolygonStamped, '/hide/keepout_zones', self._on_zones, 10)

        self.get_logger().info('R4 DynamicCostmap 시작')

    def _on_zones(self, msg: PolygonStamped):
        # TODO(R4): 폴리곤(부채꼴)을 OccupancyGrid 마스크로 래스터화 → /hide/dynamic_keepout_mask 발행
        # TODO(R4): storagy.yaml 의 costmap filter 가 이 마스크를 구독하도록 연결
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
