#pragma once

#include "comms/udp/handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include "comms/msg/sensor_board_diagnostics.hpp"

class DiagnosticsHandler : public Handler {
public:
  explicit DiagnosticsHandler(rclcpp::Node* node,
                               const std::string& topic = "sensor_board_diagnostics",
                               std::size_t queue_size = 10);

  void handle(const PBEnvelope& envelope) override;

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<comms::msg::SensorBoardDiagnostics>::SharedPtr pub_;
};
