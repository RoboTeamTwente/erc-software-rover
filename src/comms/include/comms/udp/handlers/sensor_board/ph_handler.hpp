#pragma once

#include "comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "comms/msg/sensor_board_ph_info.hpp"

class PhHandler : public Handler {
public:
  explicit PhHandler(rclcpp::Node* node,
                      const std::string& topic = "ph_data",
                      std::size_t queue_size = 10);

  void handle(const PBEnvelope& envelope) override;

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<comms::msg::SensorBoardPHInfo>::SharedPtr pub_;
};