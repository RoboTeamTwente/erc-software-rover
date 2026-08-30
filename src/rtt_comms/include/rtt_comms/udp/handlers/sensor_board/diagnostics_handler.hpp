#pragma once

#include "rtt_comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rtt_comms/msg/sensor_board_diagnostics.hpp"

class DiagnosticsHandler : public Handler {
public:
  explicit DiagnosticsHandler(rclcpp::Node* node,
                               const std::string& topic = "sensor_board_diagnostics",
                               std::size_t queue_size = 10);

  void handle(const PBEnvelope& envelope) override;

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<rtt_comms::msg::SensorBoardDiagnostics>::SharedPtr pub_;
};
