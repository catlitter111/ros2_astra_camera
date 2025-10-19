from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name', default='camera')
    camera_ns = LaunchConfiguration('camera_ns', default='camera')
    pixel_topic = LaunchConfiguration('pixel_topic', default='/target_pixel')
    publish_topic = LaunchConfiguration('publish_topic', default='/camera/target_distance')
    depth_registration = LaunchConfiguration('depth_registration', default='true')
    enable_colored_point_cloud = LaunchConfiguration('enable_colored_point_cloud', default='true')
    enable_point_cloud = LaunchConfiguration('enable_point_cloud', default='true')

    astra_launch_path = PathJoinSubstitution([
        FindPackageShare('astra_camera'), 'launch', 'astro_pro_plus.launch.xml'
    ])

    include_astra = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(astra_launch_path),
        launch_arguments={
            'camera_name': camera_name,
            'depth_registration': depth_registration,
            'enable_colored_point_cloud': enable_colored_point_cloud,
            'enable_point_cloud': enable_point_cloud,
        }.items()
    )

    pixel_node = Node(
        package='astra_pixel_distance',
        executable='pixel_distance_node.py',
        name='pixel_distance_node',
        output='screen',
        parameters=[{
            'camera_ns': camera_ns,
            'pixel_topic': pixel_topic,
            'publish_topic': publish_topic,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_ns', default_value='camera'),
        DeclareLaunchArgument('pixel_topic', default_value='/target_pixel'),
        DeclareLaunchArgument('publish_topic', default_value='/camera/target_distance'),
        DeclareLaunchArgument('depth_registration', default_value='true'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='true'),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        include_astra,
        pixel_node,
    ])