#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32

class PixelDistanceNode(Node):
    def __init__(self):
        super().__init__('pixel_distance_node')
        self.declare_parameter('camera_ns', 'camera')
        self.declare_parameter('pixel_topic', '/target_pixel')
        self.declare_parameter('publish_topic', '/camera/target_distance')
        ns = self.get_parameter('camera_ns').get_parameter_value().string_value
        pixel_topic = self.get_parameter('pixel_topic').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value

        self.depth_sub = self.create_subscription(Image, f'/{ns}/depth/image_raw', self.depth_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, f'/{ns}/depth/camera_info', self.info_cb, 10)
        self.pixel_sub = self.create_subscription(Point, pixel_topic, self.pixel_cb, 10)
        self.dist_pub = self.create_publisher(Float32, publish_topic, 10)
        self.point_pub = self.create_publisher(PointStamped, f'/{ns}/target_point', 10)

        self.depth_arr = None
        self.depth_width = None
        self.depth_height = None
        self.depth_encoding = None
        self.depth_frame_id = None
        self.fx = self.fy = self.cx = self.cy = None
        self.get_logger().info(f'PixelDistanceNode: depth=/{ns}/depth/image_raw, info=/{ns}/depth/camera_info, pixel={pixel_topic}, publish={publish_topic}')

    def depth_cb(self, msg: Image):
        self.depth_frame_id = msg.header.frame_id
        self.depth_width = msg.width
        self.depth_height = msg.height
        self.depth_encoding = msg.encoding
        if '16' in self.depth_encoding:
            arr = np.frombuffer(msg.data, dtype=np.uint16)
        elif '32' in self.depth_encoding:
            arr = np.frombuffer(msg.data, dtype=np.float32)
        else:
            self.get_logger().error(f'Unsupported depth encoding: {msg.encoding}')
            return
        try:
            self.depth_arr = arr.reshape((msg.height, msg.width))
        except ValueError:
            self.get_logger().error('Depth buffer size mismatch')
            self.depth_arr = None

    def info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def pixel_cb(self, pt: Point):
        if self.depth_arr is None or self.fx is None:
            self.get_logger().warn('Depth or CameraInfo not ready')
            return
        u = int(round(pt.x)); v = int(round(pt.y))
        if not (0 <= u < self.depth_width and 0 <= v < self.depth_height):
            self.get_logger().warn(f'Pixel out of bounds: ({u},{v})')
            return
        d = self.depth_arr[v, u]
        if '16' in self.depth_encoding:
            if d == 0:
                self.get_logger().warn('Depth=0 invalid')
                return
            z = float(d) * 0.001
        else:
            if not np.isfinite(d) or d <= 0.0:
                self.get_logger().warn('Depth invalid/NaN')
                return
            z = float(d)
        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy
        dist = float(np.sqrt(X*X + Y*Y + z*z))
        self.dist_pub.publish(Float32(data=dist))
        ps = PointStamped()
        ps.header.frame_id = self.depth_frame_id
        ps.point.x = X; ps.point.y = Y; ps.point.z = z
        self.point_pub.publish(ps)


def main():
    rclpy.init()
    node = PixelDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()