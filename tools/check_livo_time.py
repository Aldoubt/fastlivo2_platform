#!/usr/bin/env python3
import rclpy
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

        self.create_subscription(CustomMsg, '/livox/lidar', self.lidar_cb, 10)
        self.create_subscription(Imu, '/livox/imu', self.imu_cb, 100)
        self.create_subscription(Image, '/camera/image', self.img_cb, 10)

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
    rclpy.spin(node)

if __name__ == '__main__':
    main()