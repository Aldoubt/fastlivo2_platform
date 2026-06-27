#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg

def t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

class CheckTime(Node):
    def __init__(self):
        super().__init__('check_livo_time')

        self.lidar_t = None
        self.imu_t = None
        self.img_t = None

        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('image_topic', '/left_camera/image')

        lidar_topic = self.get_parameter('lidar_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        image_topic = self.get_parameter('image_topic').value

        self.get_logger().info(
            f'checking topics: lidar={lidar_topic}, imu={imu_topic}, image={image_topic}'
        )

        self.create_subscription(CustomMsg, lidar_topic, self.lidar_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 100)
        self.create_subscription(Image, image_topic, self.img_cb, 10)

        self.create_timer(1.0, self.print_status)

    def lidar_cb(self, msg):
        self.lidar_t = t(msg.header.stamp)

    def imu_cb(self, msg):
        self.imu_t = t(msg.header.stamp)

    def img_cb(self, msg):
        self.img_t = t(msg.header.stamp)

    def print_status(self):
        if self.lidar_t is None or self.imu_t is None or self.img_t is None:
            self.get_logger().info(
                f'waiting... lidar={self.lidar_t}, imu={self.imu_t}, img={self.img_t}'
            )
            return

        self.get_logger().info(
            f'img-lidar={self.img_t - self.lidar_t:+.6f}s, '
            f'imu-lidar={self.imu_t - self.lidar_t:+.6f}s, '
            f'img={self.img_t:.6f}, lidar={self.lidar_t:.6f}, imu={self.imu_t:.6f}'
        )

def main():
    rclpy.init()
    node = CheckTime()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
