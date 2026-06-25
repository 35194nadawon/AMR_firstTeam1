#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D

class GuideController(Node):
    def __init__(self):
        super().__init__('guide_controller')
        
        # Subscribe to Nav2 command
        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav2', # Adjust if Nav2 publishes to a different topic
            self.nav_callback,
            10)
            
        # Subscribe to yellow line error
        self.error_sub = self.create_subscription(
            Pose2D,
            '/guide/yellow_line_error',
            self.error_callback,
            10)
            
        # Publisher for final cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID Constants for lateral error (e_y) and angular error (e_theta)
        self.kp_y = 1.0
        self.kd_y = 0.1
        self.kp_theta = 0.5
        self.kd_theta = 0.05
        
        self.prev_e_y = 0.0
        self.prev_e_theta = 0.0
        
        # State variables
        self.current_nav_cmd = Twist()
        self.current_error = Pose2D()
        self.last_error_time = self.get_clock().now()
        
        # Timer to run control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Error threshold for Freeze (if block is lost or error is too high)
        self.max_e_y = 0.5 # meters
        self.is_frozen = False

    def nav_callback(self, msg):
        self.current_nav_cmd = msg
        
        # Check if Nav2 is ordering an emergency stop (Freeze)
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            pass

    def error_callback(self, msg):
        self.current_error = msg
        self.last_error_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_error_time).nanoseconds / 1e9
        
        # If we haven't seen an error recently (e.g., block lost for > 1 sec), we might want to stop or just rely on Nav2
        if dt > 1.0:
            self.cmd_pub.publish(self.current_nav_cmd)
            return

        e_y = self.current_error.x
        e_theta = self.current_error.theta
        
        # Exception handling: Freeze if error is abnormally large (e.g., lost path completely)
        if abs(e_y) > self.max_e_y:
            # self.get_logger().warn("Lateral error too high! Freezing robot.")
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            self.is_frozen = True
            return
            
        self.is_frozen = False
        
        # PD Control for steering correction
        de_y = e_y - self.prev_e_y
        de_theta = e_theta - self.prev_e_theta
        
        # Calculate correction (Δθ)
        delta_theta = (self.kp_y * e_y + self.kd_y * de_y) + (self.kp_theta * e_theta + self.kd_theta * de_theta)
        
        self.prev_e_y = e_y
        self.prev_e_theta = e_theta
        
        # Sensor Fusion: Inject correction into Nav2 command
        final_cmd = Twist()
        
        # Forward speed: reduce speed if steering correction is large for smoother turning
        speed_factor = max(0.2, 1.0 - abs(delta_theta) * 0.5)
        final_cmd.linear.x = self.current_nav_cmd.linear.x * speed_factor
        
        # Steering: Nav2 steering + correction
        final_cmd.angular.z = self.current_nav_cmd.angular.z + delta_theta
        
        # Limit maximum angular velocity just in case
        max_angular = 1.0
        final_cmd.angular.z = max(-max_angular, min(max_angular, final_cmd.angular.z))
        
        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    guide_controller = GuideController()
    rclpy.spin(guide_controller)
    guide_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
