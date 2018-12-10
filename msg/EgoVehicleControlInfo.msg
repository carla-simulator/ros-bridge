# This represents an info message of the ego vehicle

Header header

# vehicle maximum values
float32 max_steering_angle
float32 max_speed
float32 max_accel
float32 max_decel

# the target speed/accel values of the vehicle
float32 target_steering_angle
float32 target_speed
float32 target_speed_abs
float32 target_accel_abs
float32 target_decel_abs

# the current speed/accel values of the vehicle
float32 current_time_sec
float32 current_speed
float32 current_speed_abs
float32 current_accel

# the current control state
string status
# speed controller
float32 speed_control_accel_delta
float32 speed_control_accel_target
# acceleration controller
float32 accel_control_pedal_delta
float32 accel_control_pedal_target
# borders for lay off pedal
float32 brake_upper_border
float32 throttle_lower_border
