# Overall sensor board state (mirrors SensorDiagnostics.State in proto)
uint8 IDLE=0
uint8 OPERATING=1
uint8 CALIBRATING=2
uint8 ERROR=3

uint8 state

# Per-sensor snapshots
ImuSensorInformation imu_sensor
SensorBoardGPSInfo   gps_sensor
SensorBoardPHInfo    ph_sensor

# Board-level health
float32 board_temperature
float32 board_voltage
