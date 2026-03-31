#include "gps_payload.hpp"

#include "components/sensor_board/gps_sensor.pb.h"
#include "components/sensor_board/sensor.pb.h"

PBEnvelope make_gps_envelope() {
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

  PBEnvelope envelope;
  *envelope.mutable_gps_info() = gps;
  return envelope;
}
