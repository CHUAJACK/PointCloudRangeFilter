from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare("gz_depth_republisher")

    args = [
        DeclareLaunchArgument(
            "gz_image_topic",
            default_value="/depth_camera/depth_image",
            description="gz-transport Image topic (depth image from Gazebo)",
        ),
        DeclareLaunchArgument(
            "gz_info_topic",
            default_value="/depth_camera/camera_info",
            description="gz-transport CameraInfo topic (provides intrinsics)",
        ),
        DeclareLaunchArgument(
            "ros_topic",
            default_value="/depth_camera/points_filtered",
            description="ROS 2 PointCloud2 output topic (feed to octomap_server)",
        ),
        DeclareLaunchArgument(
            "min_range",
            default_value="0.3",
            description="Near-field cutoff in metres",
        ),
        DeclareLaunchArgument(
            "max_range",
            default_value="10.0",
            description="Far-field cutoff in metres",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_depth_optical_frame",
            description="TF frame written into the PointCloud2 header",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "config", "gz_depth_republisher.yaml"]
            ),
            description="Path to a ROS 2 parameters YAML file",
        ),
    ]

    node = Node(
        package="gz_depth_republisher",
        executable="gz_depth_republisher_node",
        name="gz_depth_republisher",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "gz_image_topic": LaunchConfiguration("gz_image_topic"),
                "gz_info_topic":  LaunchConfiguration("gz_info_topic"),
                "ros_topic":      LaunchConfiguration("ros_topic"),
                "min_range":      LaunchConfiguration("min_range"),
                "max_range":      LaunchConfiguration("max_range"),
                "frame_id":       LaunchConfiguration("frame_id"),
            },
        ],
    )

    return LaunchDescription(args + [node])
