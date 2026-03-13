# PHErrorCode
uint8 PH_NO_ERROR=0
uint8 PH_COMMUNICATION_FAILURE=1
uint8 PH_OUT_OF_RANGE=2
uint8 PH_CALIBRATION_REQUIRED=3
uint8 PH_INVALID_DATA=4
uint8 PH_PROBE_FAULT=5
uint8 PH_TEMPERATURE_SENSOR_ERROR=6

# SensorBoardPHInfo
float32 ph_value
float32 voltage
float32 temperature

SensorState state
uint8 error_code