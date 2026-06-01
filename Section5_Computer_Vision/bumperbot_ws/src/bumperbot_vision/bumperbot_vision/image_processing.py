#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from image_transport_py import ImageTransport
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import numpy as np

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_processing')

        # Declare parameters
        self.declare_parameter('camera_name', 'camera')
        self.camera_name_ = self.get_parameter('camera_name').get_parameter_value().string_value

        self.declare_parameter('lower_hsv', [79, 50, 50])
        self.declare_parameter('upper_hsv', [99, 255, 255])
        self.lower_hsv_ = self.get_parameter('lower_hsv').value
        self.upper_hsv_ = self.get_parameter('upper_hsv').value

        # Initialize helpers
        self.bridge = CvBridge()
        self.camera_info = None

        image_sub = self.create_subscription(
            Image, 
            f'{self.camera_name_}/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        
        info_sub = self.create_subscription(
            CameraInfo,
            f'{self.camera_name_}/camera_info',
            self.image_info_callback,
            qos_profile_sensor_data,
        )

        self.image_transport = ImageTransport(
            'imagetransport_pub', image_transport='compressed'
        )
        self.img_pub = self.image_transport.advertise(
            f'{self.camera_name_}/processed_image',
            qos_profile_sensor_data
        )

        self.get_logger().info(f'Image Processing node started for {self.camera_name_}')

    def image_info_callback(self, info_msg):
        self.camera_info = info_msg

    def image_callback(self, image_msg):
        if self.camera_info is None:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        
        # ============================================================
        # LAB - Color Spaces
        # ============================================================
 
        # # Convert to Grayscale — single channel, brightness only
        # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # processed_image = gray_image
 
        # # Convert to HSV
        # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # processed_image = hsv_image
 
        # # Split HSV into individual channels and visualize one
        # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # h_channel, s_channel, v_channel = cv2.split(hsv_image)
        # # Pick which channel to display: h_channel, s_channel, or v_channel
        # processed_image = v_channel
 
        # ============================================================
        # LAB — Blurring
        # ============================================================
 
        # # Gaussian Blur — smooths the image using a weighted average
        # kernel_size = 7
        # processed_image = cv2.GaussianBlur(cv_image, (kernel_size, kernel_size), 0)
 
        # # Median Blur — replaces each pixel with the median of its neighbors
        # # Excellent at removing salt-and-pepper noise while preserving edges
        # kernel_size = 7
        # processed_image = cv2.medianBlur(cv_image, kernel_size)
 
        # ============================================================
        # LAB — Drawing on Images
        # ============================================================
 
        # # Draw a line from top-left to center
        # processed_image = cv_image
        # height, width = cv_image.shape[:2]
        # cv2.line(processed_image, (0, 0), (width // 2, height // 2),
        #          color=(255, 0, 0), thickness=2)
        
        # # Draw a rectangle (simulating a bounding box)
        # top_left = (width // 4, height // 4)
        # bottom_right = (3 * width // 4, 3 * height // 4)
        # cv2.rectangle(processed_image, top_left, bottom_right,
        #               color=(0, 255, 0), thickness=2)
        
        # # Draw a filled circle at the center (simulating a detection center)
        # center = (width // 2, height // 2)
        # cv2.circle(processed_image, center, radius=5,
        #            color=(0, 0, 255), thickness=-1)
        
        # # Add a text label
        # cv2.putText(processed_image, 'Detection', (top_left[0], top_left[1] - 10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
 
        # ============================================================
        # LAB — Color Detection with Masks
        # ============================================================
 
        # # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create a mask using the HSV color range
        lower_bound = np.array(self.lower_hsv_)
        upper_bound = np.array(self.upper_hsv_)
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Morphological Opening — removes small background noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        
        # Morphological Closing — fills holes and gaps inside the object
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        
        processed_image = mask
 
        # ============================================================
        # LAB — Contour Detection
        # ============================================================
 
        # # Find contours in the clean mask
        processed_image = cv_image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
        
            # Get the minimum enclosing circle — center and radius
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
            # Only process if the detected object is big enough
            if radius > 10.0:
                # Get the bounding rectangle
                x_rect, y_rect, w, h = cv2.boundingRect(largest_contour)
        
                # Draw the bounding box (green)
                cv2.rectangle(processed_image, (x_rect, y_rect),
                              (x_rect + w, y_rect + h), (0, 255, 0), 2)
        
                # Draw the center point (red, filled)
                cv2.circle(processed_image, (int(x), int(y)), 3,
                           (0, 0, 255), -1)
 
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='passthrough')
        processed_image_msg.header = image_msg.header
        self.img_pub.publish(processed_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()