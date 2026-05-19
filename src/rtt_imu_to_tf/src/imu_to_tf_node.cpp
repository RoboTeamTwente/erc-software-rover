#include "rtt_imu_to_tf/imu_to_tf_node.h"

#include <tf2/LinearMath/Transform.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

rtt_imu_to_tf::imu_to_tf_node::imu_to_tf_node(
    const rclcpp::NodeOptions& options)
    : Node("imu_to_tf", options) {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Buffer + listener to look up the static imu→base transform
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&imu_to_tf_node::imuCallback, this, std::placeholders::_1));
}

void rtt_imu_to_tf::imu_to_tf_node::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (msg->orientation_covariance[0] == -1.0) {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "IMU orientation covariance[0] is -1; skipping TF broadcast.");
        return;
    }

    rclcpp::Time now{msg->header.stamp};

    double dt;
    if (old_time_.get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
        dt = 0;
    } else {
        dt = (now - old_time_).seconds();
    }

    // --- 1. Build odom → imu_link from IMU data ---

    tf2::Vector3 vel{
        old_vel_.x() + msg->linear_acceleration.x * dt,
        old_vel_.y() + msg->linear_acceleration.y * dt,
        old_vel_.z() + msg->linear_acceleration.z * dt,
    };

    tf2::Vector3 pos{old_pos_ + vel * dt};

    tf2::Quaternion orient{
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w,
    };

    tf2::Transform T_odom_imu{orient, pos};

    RCLCPP_INFO(get_logger(), "IMU y %lf = %lf + (%lf = %lf + %lf * %lf) * %lf",
                pos.y(), old_pos_.y(), vel.y(), old_vel_.y(),
                msg->linear_acceleration.y, dt, dt);

    old_time_ = now;
    if (dt > 0.1) return;
    old_pos_ = pos;
    old_vel_ = vel;

    // --- 2. Look up static imu_link → base_link ---
    geometry_msgs::msg::TransformStamped imu_to_base_msg;
    try {
        imu_to_base_msg =
            tf_buffer_->lookupTransform(imu_frame_,   // from  (source)
                                        base_frame_,  // to    (target)
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Could not get imu→base transform: %s",
                    ex.what());
        return;
    }

    tf2::Transform T_imu_base;
    tf2::fromMsg(imu_to_base_msg.transform, T_imu_base);

    // --- 3. Compose: odom → base_link ---
    tf2::Transform T_odom_base = T_odom_imu * T_imu_base;

    // --- 4. Publish ---
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_frame_;  // parent
    t.child_frame_id = base_frame_;   // child

    t.transform = tf2::toMsg(T_odom_base);
    tf_broadcaster_->sendTransform(t);
}

RCLCPP_COMPONENTS_REGISTER_NODE(rtt_imu_to_tf::imu_to_tf_node)
