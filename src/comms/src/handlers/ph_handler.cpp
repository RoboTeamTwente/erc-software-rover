#include "comms/udp/handlers/ph_handler.hpp"
#include "comms/udp/utils.hpp"

#include "components/sensor_board/ph_sensor.pb.h"

PhHandler::PhHandler(rclcpp::Node* node,
                       const std::string& topic,
                       std::size_t queue_size)
: node_(node)
{
  pub_ = node_->create_publisher<comms::msg::SensorBoardPHInfo>(topic, queue_size);
}

void PhHandler::handle(const PBEnvelope& envelope) {
  if (!envelope.has_ph_info()) return;

  const SensorBoardPHInfo& ph_pb = envelope.ph_info();

  comms::msg::SensorBoardPHInfo ph_ros;
  ph_ros.ph_value       = ph_pb.ph_value();
  ph_ros.voltage        = ph_pb.voltage();
  ph_ros.temperature    = ph_pb.temperature();

  ph_ros.state.state    = comms::udp::clamp_u8(static_cast<int>(ph_pb.state()));
  ph_ros.error_code     = comms::udp::clamp_u8(static_cast<int>(ph_pb.error_code()));

  pub_->publish(ph_ros);
}
