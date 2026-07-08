#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fmt(value):
    if value is None:
        return 'None'
    return f'{value:.9f}'


class CheckTime(Node):
    def __init__(self):
        super().__init__('check_livo_time')

        self.lidar_header_t = None
        self.lidar_timebase_t = None
        self.lidar_receive_t = None
        self.imu_header_t = None
        self.imu_receive_t = None
        self.img_header_t = None
        self.img_receive_t = None
        self.lidar_count = 0
        self.imu_count = 0
        self.img_count = 0

        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('image_topic', '/left_camera/image')
        self.declare_parameter('print_period_s', 1.0)

        lidar_topic = self.get_parameter('lidar_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        image_topic = self.get_parameter('image_topic').value
        print_period_s = float(self.get_parameter('print_period_s').value)

        self.get_logger().info(
            f'checking topics: lidar={lidar_topic}, imu={imu_topic}, image={image_topic}'
        )

        self.create_subscription(CustomMsg, lidar_topic, self.lidar_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 100)
        self.create_subscription(Image, image_topic, self.img_cb, 10)

        self.create_timer(print_period_s, self.print_status)

    def lidar_cb(self, msg):
        self.lidar_header_t = stamp_to_sec(msg.header.stamp)
        self.lidar_timebase_t = msg.timebase * 1e-9
        self.lidar_receive_t = stamp_to_sec(self.get_clock().now().to_msg())
        self.lidar_count += 1

    def imu_cb(self, msg):
        self.imu_header_t = stamp_to_sec(msg.header.stamp)
        self.imu_receive_t = stamp_to_sec(self.get_clock().now().to_msg())
        self.imu_count += 1

    def img_cb(self, msg):
        self.img_header_t = stamp_to_sec(msg.header.stamp)
        self.img_receive_t = stamp_to_sec(self.get_clock().now().to_msg())
        self.img_count += 1

    def print_status(self):
        if self.lidar_header_t is None or self.imu_header_t is None or self.img_header_t is None:
            self.get_logger().info(
                'waiting... '
                f'lidar_header={fmt(self.lidar_header_t)}, '
                f'lidar_timebase={fmt(self.lidar_timebase_t)}, '
                f'imu={fmt(self.imu_header_t)}, img={fmt(self.img_header_t)}'
            )
            return

        lidar_header_minus_timebase = self.lidar_header_t - self.lidar_timebase_t
        img_minus_lidar = self.img_header_t - self.lidar_timebase_t
        imu_minus_lidar = self.imu_header_t - self.lidar_timebase_t
        img_recv_minus_lidar_recv = self.img_receive_t - self.lidar_receive_t
        imu_recv_minus_lidar_recv = self.imu_receive_t - self.lidar_receive_t

        self.get_logger().info(
            f'counts lidar/imu/img={self.lidar_count}/{self.imu_count}/{self.img_count} | '
            f'header-timebase={lidar_header_minus_timebase:+.9f}s | '
            f'img-lidar_timebase={img_minus_lidar:+.9f}s | '
            f'imu-lidar_timebase={imu_minus_lidar:+.9f}s | '
            f'recv img-lidar={img_recv_minus_lidar_recv:+.6f}s | '
            f'recv imu-lidar={imu_recv_minus_lidar_recv:+.6f}s | '
            f'img={self.img_header_t:.9f}, lidar={self.lidar_timebase_t:.9f}, '
            f'imu={self.imu_header_t:.9f}'
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
