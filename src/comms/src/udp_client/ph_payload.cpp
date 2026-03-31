#include "ph_payload.hpp"

#include "components/sensor_board/ph_sensor.pb.h"
#include "components/sensor_board/sensor.pb.h"

PBEnvelope make_ph_envelope() {
  SensorBoardPHInfo ph;
  ph.set_ph_value(7.0f);
  ph.set_voltage(3.3f);
  ph.set_temperature(25.0f);

  ph.set_state(SENSOR_OPERATING);
  ph.set_error_code(PH_NO_ERROR);

  PBEnvelope envelope;
  *envelope.mutable_ph_info() = ph;
  return envelope;
}
