#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Subscribing to the downward camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/downward/image_raw',
            self.image_callback,
            10)
        
        # Publisher for the yellow line error
        self.error_publisher = self.create_publisher(Pose2D, '/guide/yellow_line_error', 10)
        
        self.cv_bridge = CvBridge()
        
        if YOLO is not None:
            # Load a YOLO model.
            # In a real scenario, this should be trained on yellow blocks.
            self.get_logger().info("Loading YOLOv8 model...")
            try:
                self.model = YOLO('yolov8n.pt') # Placeholder, use actual trained weights
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            self.get_logger().warn("Ultralytics YOLO not installed. Vision detection will be mocked.")
            self.model = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        height, width, _ = cv_image.shape
        
        # Mathematical Framework for Error Calculation:
        # e_y (Lateral Error): 
        # The center of the image horizontally is width / 2.
        # This represents the robot's center axis.
        # If the detected block has a center at (x_c, y_c), the lateral pixel error is (x_c - width / 2).
        # We can convert this pixel error to meters if we know the field of view and camera height.
        #
        # e_theta (Angular Error):
        # We extract the orientation of the bounding box. If the block is purely vertical in the image,
        # error is 0. If it tilts, we calculate the angle it makes with the vertical axis.
        
        e_y = 0.0
        e_theta = 0.0
        detected = False
        
        if self.model is not None:
            # Run YOLOv8 inference on the frame
            results = self.model(cv_image, verbose=False)
            
            # Process results (Assume class 0 or a specific class is the yellow block)
            best_box = None
            max_area = 0
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # In a real scenario, filter by class ID (e.g., box.cls == YELLOW_BLOCK_ID)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_box = box
                        
            if best_box is not None:
                x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
                x_center = (x1 + x2) / 2.0
                
                # Lateral error in pixels
                pixel_error_y = x_center - (width / 2.0)
                
                # Convert pixel error to approximate meters (Calibration needed in real life)
                m_per_pixel = 1.0 / 640.0
                e_y = pixel_error_y * m_per_pixel
                
                # Angular error calculation.
                # Standard axis-aligned bounding boxes (xyxy) don't give orientation easily.
                # In practice, an oriented bounding box (OBB) model should be used.
                if hasattr(best_box, 'obb') and best_box.obb is not None:
                    # Angle in radians
                    e_theta = float(best_box.obb.r[0].cpu().numpy())
                else:
                    # Mock angular error based on position or keep 0
                    e_theta = 0.0
                
                detected = True
        else:
            # Mock behavior if YOLO is not installed
            t = self.get_clock().now().nanoseconds / 1e9
            e_y = 0.1 * math.sin(t)
            e_theta = 0.05 * math.cos(t)
            detected = True
            
        if detected:
            error_msg = Pose2D()
            error_msg.x = float(e_y)
            error_msg.theta = float(e_theta)
            self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
