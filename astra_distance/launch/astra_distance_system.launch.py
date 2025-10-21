#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的路径
    astra_camera_pkg = get_package_share_directory('astra_camera')
    
    # 声明launch参数
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Unique camera name'
    )
    
    depth_registration_arg = DeclareLaunchArgument(
        'depth_registration',
        default_value='false',
        description='Hardware depth registration'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth stream'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )
    
    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width'
    )
    
    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height'
    )
    
    color_width_arg = DeclareLaunchArgument(
        'color_width',
        default_value='640',
        description='Color image width'
    )
    
    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='480',
        description='Color image height'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    depth_registration = LaunchConfiguration('depth_registration')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_color = LaunchConfiguration('enable_color')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    
    # 包含astro_pro_plus.launch.xml文件
    astro_pro_plus_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(astra_camera_pkg, 'launch', 'astro_pro_plus.launch.xml')
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'depth_registration': depth_registration,
            'enable_depth': enable_depth,
            'enable_color': enable_color,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'color_width': color_width,
            'color_height': color_height,
        }.items()
    )
    
    # Distance Publisher节点
    distance_publisher_node = Node(
        package='astra_distance',
        executable='distance_publisher',
        name='distance_publisher',
        output='screen',
        parameters=[
            # 可以在这里添加节点参数
        ],
        remappings=[
            # 重映射话题以匹配相机命名空间
            ('/camera/depth/image_raw', [camera_name, '/depth/image_raw']),
            ('/camera/depth/camera_info', [camera_name, '/depth/camera_info']),
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        camera_name_arg,
        depth_registration_arg,
        enable_depth_arg,
        enable_color_arg,
        depth_width_arg,
        depth_height_arg,
        color_width_arg,
        color_height_arg,
        
        # 启动文件和节点
        astro_pro_plus_launch,
        distance_publisher_node,
    ])