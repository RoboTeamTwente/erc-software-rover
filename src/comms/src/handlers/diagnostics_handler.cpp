#include "comms/udp/handlers/diagnostics_handler.hpp"
#include "comms/udp/utils.hpp"

#include "components/sensor_board/diagnostics.pb.h"
#include "components/sensor_board/imu_sensor.pb.h"
#include "components/sensor_board/gps_sensor.pb.h"
#include "components/sensor_board/ph_sensor.pb.h"

DiagnosticsHandler::DiagnosticsHandler(rclcpp::Node* node,
                                       const std::string& topic,
                                       std::size_t queue_size)
: node_(node)
{
  pub_ = node_->create_publisher<comms::msg::SensorBoardDiagnostics>(topic, queue_size);
}

void DiagnosticsHandler::handle(const PBEnvelope& envelope) {
  if (!envelope.has_sensor_diag()) return;

  const SensorBoardDiagnostics& diag_pb = envelope.sensor_diag();

  comms::msg::SensorBoardDiagnostics diag_ros;
  diag_ros.state = comms::udp::clamp_u8(static_cast<int>(diag_pb.state()));
  diag_ros.board_temperature = diag_pb.board_temperature();
  diag_ros.board_voltage     = diag_pb.board_voltage();

  // IMU
  const SensorBoardIMUInfo& imu = diag_pb.imu_sensor();
  diag_ros.imu_sensor.accel_x      = imu.accel_x();
  diag_ros.imu_sensor.accel_y      = imu.accel_y();
  diag_ros.imu_sensor.accel_z      = imu.accel_z();
  diag_ros.imu_sensor.gyro_x       = imu.gyro_x();
  diag_ros.imu_sensor.gyro_y       = imu.gyro_y();
  diag_ros.imu_sensor.gyro_z       = imu.gyro_z();
  diag_ros.imu_sensor.mag_x        = imu.mag_x();
  diag_ros.imu_sensor.mag_y        = imu.mag_y();
  diag_ros.imu_sensor.mag_z        = imu.mag_z();
  diag_ros.imu_sensor.is_calibrated = imu.is_calibrated();
  diag_ros.imu_sensor.state.state  = comms::udp::clamp_u8(static_cast<int>(imu.state()));
  diag_ros.imu_sensor.error_code   = comms::udp::clamp_u8(static_cast<int>(imu.error_code()));

  // GPS
  const SensorBoardGPSInfo& gps = diag_pb.gps_sensor_1();
  diag_ros.gps_sensor.latitude      = gps.latitude();
  diag_ros.gps_sensor.longitude     = gps.longitude();
  diag_ros.gps_sensor.altitude      = gps.altitude();
  diag_ros.gps_sensor.speed         = gps.speed();
  diag_ros.gps_sensor.heading       = gps.heading();
  diag_ros.gps_sensor.hdop          = gps.hdop();
  diag_ros.gps_sensor.vdop          = gps.vdop();
  diag_ros.gps_sensor.satellites    = gps.satellites();
  diag_ros.gps_sensor.fix_quality   = comms::udp::clamp_u8(static_cast<int>(gps.fix_quality()));
  diag_ros.gps_sensor.state.state   = comms::udp::clamp_u8(static_cast<int>(gps.state()));
  diag_ros.gps_sensor.error_code    = comms::udp::clamp_u8(static_cast<int>(gps.error_code()));
  diag_ros.gps_sensor.utc_timestamp = gps.utc_timestamp();

  // PH
  const SensorBoardPHInfo& ph = diag_pb.ph_sensor();
  diag_ros.ph_sensor.ph_value    = ph.ph_value();
  diag_ros.ph_sensor.voltage     = ph.voltage();
  diag_ros.ph_sensor.temperature = ph.temperature();
  diag_ros.ph_sensor.state.state = comms::udp::clamp_u8(static_cast<int>(ph.state()));
  diag_ros.ph_sensor.error_code  = comms::udp::clamp_u8(static_cast<int>(ph.error_code()));

  pub_->publish(diag_ros);
}
