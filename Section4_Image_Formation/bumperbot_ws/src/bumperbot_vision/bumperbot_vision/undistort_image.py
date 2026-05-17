#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from image_transport_py import ImageTransport
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel 
import numpy as np

class UndistortImage(Node):
    def __init__(self):
        super().__init__('undistort_image')

        # Declare parameters
        self.declare_parameter('camera_name', 'left_camera')
        self.camera_name_ = self.get_parameter('camera_name').get_parameter_value().string_value

        # Initialize helpers
        self.bridge = CvBridge()
        self.model = PinholeCameraModel()

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
            f'{self.camera_name_}/image_rect',
            qos_profile_sensor_data
        )

        self.camera_info = None
        self.get_logger().info(f'Undistort node started for {self.camera_name_}')

    def image_info_callback(self, info_msg):
        self.camera_info = info_msg

    def image_callback(self, image_msg):
        if self.camera_info is None:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        self.model.from_camera_info(self.camera_info)

        image_undistort = np.zeros_like(cv_image)
        self.model.rectify_image(cv_image, image_undistort)
        undistort_msg = self.bridge.cv2_to_imgmsg(image_undistort, encoding=image_msg.encoding)
        undistort_msg.header = image_msg.header
        self.undistort_image_pub_.publish(undistort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UndistortImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()