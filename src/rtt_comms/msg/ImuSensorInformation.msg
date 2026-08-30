# Note: Might want to use pre-made ROS messages from sensor_msgs package instead of custom ones, but for now we can use these as placeholders until we decide on the final message structure.

# IMU error codes (match protobuf exactly)
uint8 IMU_NO_ERROR=0
uint8 IMU_COMMUNICATION_FAILURE=1
uint8 IMU_CALIBRATION_REQUIRED=2
uint8 IMU_CALIBRATION_FAILED=3
uint8 IMU_INVALID_DATA=4
uint8 IMU_SENSOR_FAULT=5
uint8 IMU_GYROSCOPE_ERROR=6
uint8 IMU_MAGNETOMETER_ERROR=7
uint8 IMU_ACCELEROMETER_ERROR=8

# 3-Axis Accelerometer
float32 accel_x
float32 accel_y
float32 accel_z

# 3-Axis Gyroscope
float32 gyro_x
float32 gyro_y
float32 gyro_z

# 3-Axis Magnetometer
float32 mag_x
float32 mag_y
float32 mag_z

bool is_calibrated

SensorState state
uint8 error_code


