#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from octomap_msgs.msg import Octomap

from tf2_ros import Buffer, TransformListener, TransformException

# Python wrapper around the octomap C++ library (pip install octomap-python)
import octomap


class DepthReconstruction(Node):

    def __init__(self, name):
        super().__init__(name)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # Octree with 5 cm resolution (the Python wrapper only exposes
        # the plain OcTree, so no per-voxel color like ColorOcTree in C++)
        self.resolution_ = 0.05
        self.octree_ = octomap.OcTree(self.resolution_)

        # Our global map: Nx3 points + Nx3 colors accumulated over time
        self.merged_points_ = np.empty((0, 3), dtype=np.float32)
        self.merged_colors_ = np.empty((0, 3), dtype=np.uint8)

        # No point_cloud_transport in Python: plain pub / sub on PointCloud2
        self.cloud_sub_ = self.create_subscription(
            PointCloud2, "camera/points", self.cloud_callback, 10)
        self.merged_cloud_pub_ = self.create_publisher(
            PointCloud2, "merged_cloud", 10)
        self.octomap_pub_ = self.create_publisher(
            Octomap, "octomap", 10)

    def cloud_callback(self, cloud_msg):
        # Convert the ROS PointCloud2 message to numpy arrays
        cloud = point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        if cloud.shape[0] == 0:
            return

        points = np.column_stack(
            (cloud["x"], cloud["y"], cloud["z"])).astype(np.float32)

        # The "rgb" field is a float32 whose raw bits hold 0x00RRGGBB:
        # reinterpret the bits as uint32 and unpack the three channels
        rgb_bits = cloud["rgb"].copy().view(np.uint32)
        colors = np.column_stack((
            (rgb_bits >> 16) & 0xFF,   # R
            (rgb_bits >> 8) & 0xFF,    # G
            rgb_bits & 0xFF            # B
        )).astype(np.uint8)

        # Filter out points that are too far away (PassThrough on z)
        mask = (points[:, 2] >= 0.1) & (points[:, 2] <= 5.0)
        points = points[mask]
        colors = colors[mask]

        # Transform the point cloud from the camera frame to the global frame
        try:
            transform = self.tf_buffer_.lookup_transform(
                "odom", cloud_msg.header.frame_id,
                Time.from_msg(cloud_msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException:
            self.get_logger().error("Failed to transform point cloud!")
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        transformed_points = points @ rotation.T + translation

        # Accumulate the transformed point cloud into our global map
        self.merged_points_ = np.vstack((self.merged_points_, transformed_points))
        self.merged_colors_ = np.vstack((self.merged_colors_, colors))

        # Downsample the merged point cloud (VoxelGrid filter)
        self.merged_points_, self.merged_colors_ = self.voxel_grid_filter(
            self.merged_points_, self.merged_colors_, self.resolution_)

        # Convert the merged cloud back to a ROS PointCloud2 message
        merged_msg = self.to_point_cloud2(
            self.merged_points_, self.merged_colors_, "odom")
        self.merged_cloud_pub_.publish(merged_msg)

        # Update the OctoMap, convert it to a ROS message and publish
        for point in self.merged_points_:
            self.octree_.updateNode(
                point.astype(np.float64), True, lazy_eval=True)
        self.octree_.updateInnerOccupancy()

        octomap_msg = Octomap()
        octomap_msg.header.frame_id = "map"
        octomap_msg.header.stamp = self.get_clock().now().to_msg()
        octomap_msg.binary = True
        octomap_msg.id = "OcTree"
        octomap_msg.resolution = self.resolution_
        # writeBinary() returns the octree serialized as a file:
        # a small ASCII header ending with the "data" keyword, followed
        # by the raw binary payload that the Octomap message expects
        serialized = self.octree_.writeBinary()
        payload = serialized[serialized.find(b"\ndata\n") + len(b"\ndata\n"):]
        octomap_msg.data = np.frombuffer(payload, dtype=np.int8).tolist()
        self.octomap_pub_.publish(octomap_msg)

    @staticmethod
    def quaternion_to_matrix(x, y, z, w):
        # Standard quaternion -> 3x3 rotation matrix conversion
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)

    @staticmethod
    def voxel_grid_filter(points, colors, leaf_size):
        # Assign every point to a voxel of size leaf_size
        voxel_indices = np.floor(points / leaf_size).astype(np.int64)

        # Group the points that fall in the same voxel
        _, inverse, counts = np.unique(
            voxel_indices, axis=0, return_inverse=True, return_counts=True)

        # Replace each group with the centroid of its points (average
        # position and color), exactly like PCL's VoxelGrid filter
        point_sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
        color_sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
        np.add.at(point_sums, inverse, points)
        np.add.at(color_sums, inverse, colors)

        centroids = (point_sums / counts[:, None]).astype(np.float32)
        mean_colors = (color_sums / counts[:, None]).astype(np.uint8)
        return centroids, mean_colors

    def to_point_cloud2(self, points, colors, frame_id):
        # Pack the r, g, b channels back into a single float32 field
        rgb_bits = (colors[:, 0].astype(np.uint32) << 16) | \
                   (colors[:, 1].astype(np.uint32) << 8) | \
                   colors[:, 2].astype(np.uint32)
        rgb_float = rgb_bits.view(np.float32)

        # Build a structured array matching the PointCloud2 memory layout
        cloud = np.empty(points.shape[0], dtype=[
            ("x", np.float32), ("y", np.float32),
            ("z", np.float32), ("rgb", np.float32)])
        cloud["x"] = points[:, 0]
        cloud["y"] = points[:, 1]
        cloud["z"] = points[:, 2]
        cloud["rgb"] = rgb_float

        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = cloud.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = cloud.dtype.itemsize
        msg.row_step = msg.point_step * cloud.shape[0]
        msg.is_dense = True
        msg.data = cloud.tobytes()
        return msg


def main():
    rclpy.init()
    node = DepthReconstruction("depth_reconstruction")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()