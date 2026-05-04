#pragma once

#include <rcl/time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rtt_imu_to_tf {
class imu_to_tf_node : public rclcpp::Node {
   public:
    imu_to_tf_node(const rclcpp::NodeOptions& options);

   private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::string odom_frame_, base_frame_, imu_frame_;

    // Broadcasting
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2::Vector3 old_pos_, old_vel_;
    rclcpp::Time old_time_{0, 0, RCL_CLOCK_UNINITIALIZED};

    // Listening (to look up imu_link → base_link)

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
}  // namespace rtt_imu_to_tf
