#pragma once

#include "rtt_comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rtt_comms/msg/sensor_board_gps_info.hpp"

class GpsHandler : public Handler {
public:
  explicit GpsHandler(rclcpp::Node* node,
                      const std::string& topic = "gps_data",
                      std::size_t queue_size = 10);

  void handle(const PBEnvelope& envelope) override;

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<rtt_comms::msg::SensorBoardGPSInfo>::SharedPtr pub_;
};
