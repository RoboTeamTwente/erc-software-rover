# GPSFixQuality
uint8 NO_FIX=0
uint8 GPS_FIX=1
uint8 DGPS_FIX=2
uint8 PPS_FIX=3
uint8 RTK_FIX=4
uint8 RTK_FLOAT=5

# GPSErrorCode
uint8 GPS_NO_ERROR=0
uint8 GPS_COMMUNICATION_FAILURE=1
uint8 GPS_INVALID_DATA = 2
uint8 GPS_ANTENNA_FAULT=3
uint8 GPS_LOW_SIGNAL_QUALITY=4

# SensorBoardGPSInfo message definition
float64 latitude
float64 longitude
float32 altitude

float32 speed
float32 heading

float32 hdop
float32 vdop
int32 satellites

uint8 fix_quality

SensorState state
uint8 error_code

int64 utc_timestamp
