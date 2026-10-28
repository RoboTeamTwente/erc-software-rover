#!/usr/bin/env python3
"""Adds covariance to the raw Gazebo wheel odometry and IMU messages.

Gazebo publishes zero covariance. The EKF weights every source by its covariance, so zero
means "trust completely". The real rover needs the same step (the board IMU message has no
covariance at all), so the numbers below double as a first template for it.

/wheel/odom_raw -> /wheel/odom
/imu/data_raw   -> /imu/data
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def diag6(values):
    m = [0.0] * 36
    for i, v in enumerate(values):
        m[i * 7] = float(v)
    return m


def diag3(values):
    m = [0.0] * 9
    for i, v in enumerate(values):
        m[i * 4] = float(v)
    return m


class CovarianceStamper(Node):
    def __init__(self):
        super().__init__('covariance_stamper')
        # variances (stddev squared). Placeholders, tune with real data.
        self.declare_parameter('odom_pose_var', [1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2])
        self.declare_parameter('odom_twist_var', [1e-2, 1e-1, 1e6, 1e6, 1e6, 2e-2])
        self.declare_parameter('imu_orientation_var', [1e-2, 1e-2, 1e-2])
        self.declare_parameter('imu_gyro_var', [1e-5, 1e-5, 1e-5])
        self.declare_parameter('imu_accel_var', [4e-4, 4e-4, 4e-4])

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(Odometry, '/wheel/odom_raw', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)

    def on_odom(self, msg):
        msg.pose.covariance = diag6(self.get_parameter('odom_pose_var').value)
        msg.twist.covariance = diag6(self.get_parameter('odom_twist_var').value)
        self.odom_pub.publish(msg)

    def on_imu(self, msg):
        msg.orientation_covariance = diag3(self.get_parameter('imu_orientation_var').value)
        msg.angular_velocity_covariance = diag3(self.get_parameter('imu_gyro_var').value)
        msg.linear_acceleration_covariance = diag3(self.get_parameter('imu_accel_var').value)
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = CovarianceStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
