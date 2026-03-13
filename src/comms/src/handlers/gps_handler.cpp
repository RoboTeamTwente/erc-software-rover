#include "comms/udp/handlers/gps_handler.hpp"
#include "comms/udp/utils.hpp"

#include "components/sensor_board/gps_sensor.pb.h"

GpsHandler::GpsHandler(rclcpp::Node* node,
                       const std::string& topic,
                       std::size_t queue_size)
: node_(node)
{
  pub_ = node_->create_publisher<comms::msg::SensorBoardGPSInfo>(topic, queue_size);
}

void GpsHandler::handle(const PBEnvelope& envelope) {
  if (!envelope.has_gps_info()) return;

  const SensorBoardGPSInfo& gps_pb = envelope.gps_info();

  comms::msg::SensorBoardGPSInfo gps_ros;
  gps_ros.latitude      = gps_pb.latitude();
  gps_ros.longitude     = gps_pb.longitude();
  gps_ros.altitude      = gps_pb.altitude();

  gps_ros.speed         = gps_pb.speed();
  gps_ros.heading       = gps_pb.heading();

  gps_ros.hdop          = gps_pb.hdop();
  gps_ros.vdop          = gps_pb.vdop();
  gps_ros.satellites    = gps_pb.satellites();

  gps_ros.fix_quality   = comms::udp::clamp_u8(static_cast<int>(gps_pb.fix_quality()));
  gps_ros.state.state   = comms::udp::clamp_u8(static_cast<int>(gps_pb.state()));
  gps_ros.error_code    = comms::udp::clamp_u8(static_cast<int>(gps_pb.error_code()));

  gps_ros.utc_timestamp = gps_pb.utc_timestamp();

  pub_->publish(gps_ros);
}
