#include "comms/udp/handlers/imu_handler.hpp"
#include "comms/udp/utils.hpp"

#include "components/sensor_board/imu_sensor.pb.h"

ImuHandler::ImuHandler(rclcpp::Node* node,
                       const std::string& topic,
                       std::size_t queue_size)
: node_(node)
{
  pub_ = node_->create_publisher<comms::msg::ImuSensorInformation>(topic, queue_size);
}

void ImuHandler::handle(const PBEnvelope& envelope) {
  if (!envelope.has_imu_info()) return;

  const SensorBoardIMUInfo& imu_pb = envelope.imu_info();

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

  imu_ros.is_calibrated  = imu_pb.is_calibrated();
  imu_ros.state.state    = comms::udp::clamp_u8(static_cast<int>(imu_pb.state()));
  imu_ros.error_code     = comms::udp::clamp_u8(static_cast<int>(imu_pb.error_code()));

  pub_->publish(imu_ros);
}
