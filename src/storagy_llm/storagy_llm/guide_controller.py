#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class GuideController(Node):
    """Small bridge node for the integrated guide flow.

    The guide_nav_node owns Nav2 goal execution. This node observes guide mission
    completion and releases LLM/wander coordination state for the rest of the
    system.
    """

    def __init__(self):
        super().__init__('guide_controller')
        self.llm_active_pub = self.create_publisher(Bool, '/llm_active', 10)
        self.event_pub = self.create_publisher(String, '/robot_events', 10)
        self.create_subscription(
            Bool, '/guide/mission_done', self._on_guide_done, 10)
        self.get_logger().info('guide_controller ready')

    def _publish_event(self, text: str):
        self.event_pub.publish(String(data=text))

    def _on_guide_done(self, msg: Bool):
        if not msg.data:
            return
        self.llm_active_pub.publish(Bool(data=False))
        self._publish_event('Guide arrived at target; waiting for mission completion')
        self.get_logger().info('Guide mission_done received; llm_active=false')


def main(args=None):
    rclpy.init(args=args)
    node = GuideController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
