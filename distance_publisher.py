#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 订阅坐标点
        self.point_subscription = self.create_subscription(
            PointStamped,
            '/target_point',
            self.point_callback,
            10
        )
        
        # 订阅深度图像
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # 订阅相机信息
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布距离信息
        self.distance_publisher = self.create_publisher(
            Float32,
            '/point_distance',
            10
        )
        
        # 发布带距离信息的点
        self.point_with_distance_publisher = self.create_publisher(
            PointStamped,
            '/point_with_distance',
            10
        )
        
        # 存储最新的深度图像和相机信息
        self.latest_depth_image = None
        self.camera_info = None
        self.target_point = None
        
        self.get_logger().info('Distance Publisher Node已启动')
        self.get_logger().info('订阅话题: /target_point (geometry_msgs/PointStamped)')
        self.get_logger().info('订阅话题: /camera/depth/image_raw (sensor_msgs/Image)')
        self.get_logger().info('订阅话题: /camera/depth/camera_info (sensor_msgs/CameraInfo)')
        self.get_logger().info('发布话题: /point_distance (std_msgs/Float32)')
        self.get_logger().info('发布话题: /point_with_distance (geometry_msgs/PointStamped)')
    
    def camera_info_callback(self, msg):
        """接收相机信息"""
        self.camera_info = msg
        
    def depth_callback(self, msg):
        """接收深度图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            if msg.encoding == "16UC1":
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            elif msg.encoding == "32FC1":
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            else:
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                
            # 如果有目标点，计算距离
            if self.target_point is not None:
                self.calculate_and_publish_distance()
                
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {str(e)}')
    
    def point_callback(self, msg):
        """接收目标坐标点"""
        self.target_point = msg
        self.get_logger().info(f'接收到目标点: x={msg.point.x:.2f}, y={msg.point.y:.2f}')
        
        # 如果有深度图像，立即计算距离
        if self.latest_depth_image is not None:
            self.calculate_and_publish_distance()
    
    def calculate_and_publish_distance(self):
        """计算并发布距离信息"""
        if self.target_point is None or self.latest_depth_image is None:
            return
            
        try:
            # 获取图像坐标（假设输入的点是像素坐标）
            x = int(self.target_point.point.x)
            y = int(self.target_point.point.y)
            
            # 检查坐标是否在图像范围内
            height, width = self.latest_depth_image.shape[:2]
            if x < 0 or x >= width or y < 0 or y >= height:
                self.get_logger().warn(f'坐标超出图像范围: ({x}, {y}), 图像大小: {width}x{height}')
                return
            
            # 获取深度值
            depth_value = self.latest_depth_image[y, x]
            
            # 处理不同的深度编码
            if self.latest_depth_image.dtype == np.uint16:
                # 16位深度图，通常以毫米为单位
                distance_mm = float(depth_value)
                distance_m = distance_mm / 1000.0  # 转换为米
            elif self.latest_depth_image.dtype == np.float32:
                # 32位浮点深度图，通常以米为单位
                distance_m = float(depth_value)
            else:
                distance_m = float(depth_value) / 1000.0
            
            # 检查深度值是否有效
            if distance_m <= 0 or np.isnan(distance_m) or np.isinf(distance_m):
                self.get_logger().warn(f'无效的深度值: {distance_m}')
                return
            
            # 发布距离信息
            distance_msg = Float32()
            distance_msg.data = distance_m
            self.distance_publisher.publish(distance_msg)
            
            # 创建带距离信息的点消息
            point_with_distance = PointStamped()
            point_with_distance.header = self.target_point.header
            point_with_distance.header.stamp = self.get_clock().now().to_msg()
            point_with_distance.point.x = self.target_point.point.x
            point_with_distance.point.y = self.target_point.point.y
            point_with_distance.point.z = distance_m  # 将距离存储在z坐标中
            
            self.point_with_distance_publisher.publish(point_with_distance)
            
            self.get_logger().info(f'像素坐标 ({x}, {y}) 的距离: {distance_m:.3f} 米')
            
        except Exception as e:
            self.get_logger().error(f'距离计算错误: {str(e)}')
    
    def pixel_to_3d_point(self, x, y, depth):
        """将像素坐标和深度转换为3D点（如果有相机内参）"""
        if self.camera_info is None:
            return None
            
        try:
            # 获取相机内参
            fx = self.camera_info.k[0]  # K[0,0]
            fy = self.camera_info.k[4]  # K[1,1]
            cx = self.camera_info.k[2]  # K[0,2]
            cy = self.camera_info.k[5]  # K[1,2]
            
            # 计算3D坐标
            z = depth
            x_3d = (x - cx) * z / fx
            y_3d = (y - cy) * z / fy
            
            return (x_3d, y_3d, z)
            
        except Exception as e:
            self.get_logger().error(f'3D点计算错误: {str(e)}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    distance_publisher = DistancePublisher()
    
    try:
        rclpy.spin(distance_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        distance_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()