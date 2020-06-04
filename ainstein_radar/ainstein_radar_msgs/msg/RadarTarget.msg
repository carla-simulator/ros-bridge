# This message describes a target (detection) from a RADAR sensor.

uint16 target_id    # ID of the target, as set by the sensor
float64 snr         # Signal-to-noise ratio
float64 range       # Distance from sensor to target along sensor x-axis
float64 speed       # Speed (range rate of change) of target along sensor x-axis
float64 azimuth     # Angle of target relative to sensor within x-y plane
float64 elevation   # Angle of target relative to sensor within y-z plane
