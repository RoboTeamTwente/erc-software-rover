#pragma once

#include "comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "comms/msg/imu_sensor_information.hpp"

#include "components/sensorboard/imu_sensor.pb.h"
#include "components/common/sensor.pb.h"

class ImuHandler : public Handler {
public:
  explicit ImuHandler(rclcpp::Node* node,
                      const std::string& topic = "imu_data",
                      std::size_t queue_size = 10)
  : node_(node)
  {
    pub_ = node_->create_publisher<comms::msg::ImuSensorInformation>(topic, queue_size);
  }

  void handle(const uint8_t* payload, std::size_t len) override {
    IMUSensorInformation imu_pb;
    if (!imu_pb.ParseFromArray(payload, static_cast<int>(len))) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "IMU handler: Failed to parse protobuf message (%zu bytes)", len);
      return;
    }

    comms::msg::ImuSensorInformation imu_ros;
    imu_ros.accel_x = imu_pb.accel_x();
    imu_ros.accel_y = imu_pb.accel_y();
    imu_ros.accel_z = imu_pb.accel_z();

    imu_ros.gyro_x = imu_pb.gyro_x();
    imu_ros.gyro_y = imu_pb.gyro_y();
    imu_ros.gyro_z = imu_pb.gyro_z();

    imu_ros.mag_x = imu_pb.mag_x();
    imu_ros.mag_y = imu_pb.mag_y();
    imu_ros.mag_z = imu_pb.mag_z();

    imu_ros.is_calibrated = imu_pb.is_calibrated();

    // SensorState.msg ends with "uint8 state"
    imu_ros.state.state = clamp_u8(static_cast<int>(imu_pb.state()));
    imu_ros.error_code  = clamp_u8(static_cast<int>(imu_pb.error_code()));

    pub_->publish(imu_ros);
  }

private:
  static uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
  }

  rclcpp::Node* node_;
  rclcpp::Publisher<comms::msg::ImuSensorInformation>::SharedPtr pub_;
};