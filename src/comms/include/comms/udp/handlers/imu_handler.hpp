#pragma once

#include "comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "comms/msg/imu_sensor_information.hpp"

class ImuHandler : public Handler {
public:
  explicit ImuHandler(rclcpp::Node* node,
                      const std::string& topic = "imu_data",
                      std::size_t queue_size = 10);

  void handle(const PBEnvelope& envelope) override;

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<comms::msg::ImuSensorInformation>::SharedPtr pub_;
};
