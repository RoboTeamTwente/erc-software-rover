#include "rtt_imu_to_tf/imu_to_tf_node.h"

#include <rclcpp_components/register_node_macro.hpp>

rtt_imu_to_tf::imu_to_tf_node::imu_to_tf_node(
    const rclcpp::NodeOptions& options)
    : Node("imu_to_tf", options) {
    RCLCPP_INFO(get_logger(), "hi world");
}

RCLCPP_COMPONENTS_REGISTER_NODE(rtt_imu_to_tf::imu_to_tf_node)
