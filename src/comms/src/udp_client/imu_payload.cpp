#include "comms/udp_client/imu_payload.hpp"

#include "components/sensor_board/imu_sensor.pb.h"
#include "components/sensor_board/sensor.pb.h"

PBEnvelope make_imu_envelope() {
  SensorBoardIMUInfo imu;
  imu.set_accel_x(0.12f);
  imu.set_accel_y(-9.81f);
  imu.set_accel_z(0.05f);

  imu.set_gyro_x(0.01f);
  imu.set_gyro_y(0.02f);
  imu.set_gyro_z(0.03f);

  imu.set_mag_x(30.0f);
  imu.set_mag_y(1.5f);
  imu.set_mag_z(-44.2f);

  imu.set_is_calibrated(true);
  imu.set_state(SENSOR_OPERATING);
  imu.set_error_code(IMU_NO_ERROR);

  PBEnvelope envelope;
  *envelope.mutable_imu_info() = imu;
  return envelope;
}
