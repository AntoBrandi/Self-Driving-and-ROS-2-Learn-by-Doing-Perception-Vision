from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    image_rectify_right = Node(
        package="bumperbot_vision",
        executable="undistort_image",
        name="undistort_image_right",
        output="screen",
        parameters=[{"camera_name": "right_camera", 
                     "use_sim_time": use_sim_time}],
    )

    image_rectify_left = Node(
        package="bumperbot_vision",
        executable="undistort_image",
        name="undistort_image_left",
        output="screen",
        parameters=[{"camera_name": "left_camera", 
                     "use_sim_time": use_sim_time}],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[{"use_mag": False, 
                     "world_frame":"enu", 
                     "publish_tf": False,
                     "use_sim_time": use_sim_time}],
        remappings=[("imu/data_raw", "/imu/out"),
                    ("imu/data", "/imu/filtered")]
    )

    rtabmap_odometry = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="stereo_odometry",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_vision"), "config", "rtabmap.yaml"),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("left/image_rect", "/left_camera/image_rect"),
            ("right/image_rect", "/right_camera/image_rect"),
            ("left/camera_info", "/left_camera/camera_info"),
            ("right/camera_info", "/right_camera/camera_info"),
            ("odom", "/camera/odom"),
            ("imu", "/imu/filtered")
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_filter,
        rtabmap_odometry,
        image_rectify_right,
        image_rectify_left,
    ])