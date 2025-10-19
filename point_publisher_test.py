#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time

class PointPublisherTest(Node):
    def __init__(self):
        super().__init__('point_publisher_test')
        
        # 创建发布器
        self.publisher = self.create_publisher(
            PointStamped,
            '/target_point',
            10
        )
        
        # 创建定时器，每2秒发布一次测试点
        self.timer = self.create_timer(2.0, self.publish_test_point)
        
        # 测试点列表（像素坐标）
        self.test_points = [
            (320, 240),  # 图像中心
            (160, 120),  # 左上区域
            (480, 360),  # 右下区域
            (100, 200),  # 左侧
            (540, 280),  # 右侧
        ]
        
        self.point_index = 0
        
        self.get_logger().info('Point Publisher Test Node已启动')
        self.get_logger().info('将每2秒发布一个测试点到 /target_point')
    
    def publish_test_point(self):
        """发布测试点"""
        if self.point_index >= len(self.test_points):
            self.point_index = 0
        
        x, y = self.test_points[self.point_index]
        
        # 创建PointStamped消息
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_depth_optical_frame"
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = 0.0  # 深度将由distance_publisher计算
        
        self.publisher.publish(point_msg)
        
        self.get_logger().info(f'发布测试点 {self.point_index + 1}: ({x}, {y})')
        
        self.point_index += 1

def main(args=None):
    rclpy.init(args=args)
    
    point_publisher_test = PointPublisherTest()
    
    try:
        rclpy.spin(point_publisher_test)
    except KeyboardInterrupt:
        pass
    finally:
        point_publisher_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()