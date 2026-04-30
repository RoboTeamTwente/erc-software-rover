#pragma once

#include <rclcpp/rclcpp.hpp>

namespace rtt_imu_to_tf {

class imu_to_tf_node : public rclcpp::Node {
   public:
    imu_to_tf_node(const rclcpp::NodeOptions& options);
};

}  // namespace rtt_imu_to_tf
