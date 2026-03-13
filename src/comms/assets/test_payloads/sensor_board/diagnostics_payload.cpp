#include "test_payloads/sensor_board/diagnostics_payload.hpp"

#include "components/sensor_board/diagnostics.pb.h"
#include "components/sensor_board/imu_sensor.pb.h"
#include "components/sensor_board/gps_sensor.pb.h"
#include "components/sensor_board/ph_sensor.pb.h"
#include "components/sensor_board/sensor.pb.h"

PBEnvelope make_diagnostics_envelope() {
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

  SensorBoardGPSInfo gps;
  gps.set_latitude(52.2297);
  gps.set_longitude(21.0122);
  gps.set_altitude(110.0f);
  gps.set_speed(0.5f);
  gps.set_heading(90.0f);
  gps.set_hdop(1.2f);
  gps.set_vdop(1.8f);
  gps.set_satellites(9);
  gps.set_fix_quality(GPS_FIX);
  gps.set_state(SENSOR_OPERATING);
  gps.set_error_code(GPS_NO_ERROR);
  gps.set_utc_timestamp(0);

  SensorBoardPHInfo ph;
  ph.set_ph_value(7.0f);
  ph.set_voltage(3.3f);
  ph.set_temperature(25.0f);
  ph.set_state(SENSOR_OPERATING);
  ph.set_error_code(PH_NO_ERROR);

  SensorBoardDiagnostics diag;
  diag.set_state(SensorBoardDiagnostics::OPERATING);
  *diag.mutable_imu_sensor()  = imu;
  *diag.mutable_gps_sensor_1() = gps;
  *diag.mutable_ph_sensor()   = ph;
  diag.set_board_temperature(38.5f);
  diag.set_board_voltage(5.0f);

  PBEnvelope envelope;
  *envelope.mutable_sensor_diag() = diag;
  return envelope;
}
