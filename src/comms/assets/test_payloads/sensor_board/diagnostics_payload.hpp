#pragma once
#include "components/common/envelope.pb.h"

// Returns a PBEnvelope with sensor_diag populated with test data.
PBEnvelope make_diagnostics_envelope();
