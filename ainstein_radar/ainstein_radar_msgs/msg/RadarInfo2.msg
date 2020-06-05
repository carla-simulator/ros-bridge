# This message defines meta information for a radar sensor. It should
# be in a radar namespace on topic "radar_info" and accompanied by up
# to three radar topics named:
#
#   targets/raw      - raw (untracked) radar detections
#   targets/tracked  - tracked radar detections
#   alarms           - alarms based on detections, eg BSD
#
# In general, all Ainstein radars output raw detections and may also
# output tracked detections if trackign algorithms are implemented in
# firmware. Tracking from raw data is also available in ROS; see the
# ainstein_radar_filters package for more information. Most radars do
# not output alarms as this is specific to the automotive use case,
# however this message type is retained for the time being for backward
# compatibility.

#######################################################################
#                     Data acquisition info                           #
#######################################################################

# Time of data acquisition, radar coordinate frame ID
std_msgs/Header header    # Header timestamp should be acquisition time of data
                 # Header frame_id should be radar sensing frame
                 # origin of frame should be center of sensor
                 # +x should point radially outwards from the radar
                 # +y should point to complete a right-handed frame
                 # +z should point upwards

#######################################################################
#                    General sensor properties                        #
#######################################################################

# The nominal update rate of the sensor reported in Hz.
float64 update_rate

# The maximum number of detections (targets) the sensors can report.
uint16 max_num_targets

#######################################################################
#                    Physical sensing limits                          #
#######################################################################
# These are limits imposed by the antenna hardware and/or cutoffs set #
# in the detection processing firmware. They come from sensor data    #
# sheets and must be updated with each hardware revision as necessary.#
#######################################################################

# The minimum and maximum range, in meters, of detections (targets)
# reported by the sensor.
float64 range_min
float64 range_max

# The minimum and maximum speed, in meters per second, of detections
# (targets) reported by the sensor.
float64 speed_min
float64 speed_max

# The minimum and maximum azimuth angle, in degrees.
float64 azimuth_min
float64 azimuth_max

# The minimum and maximum azimuth angle, in degrees.
float64 elevation_min
float64 elevation_max

#######################################################################
#                    Physical sensing precision                       #
#######################################################################
# These are also imposed by the antenna hardware and/or set in the    #
# detection processing firmware. They also come from sensor data      #
# sheets and must be updated with each hardware revision as necessary.#
#######################################################################

# Range resolution, in meters. The resolution is defined as the minimum
# distance between two objects which results in distinct detections.
float64 range_resolution

# Range accuracy, in meters. The accuracy is defined as the precision
# with which range of a detection is reported.
float64 range_accuracy

# Speed resolution, in meters per second. The resolution is defined as
# the minimum speed difference between two objects which results in
# distinct detections.
float64 speed_resolution

# Speed accuracy, in meters per second. The accuracy is defined as the
# precision with which speed of a detection is reported.
float64 speed_accuracy

# Azimuth angle resolution, in degrees. The resolution is defined as
# the minimum azimuth angle between two objects which results in
# distinct detections.
float64 azimuth_resolution

# Azimuth angle accuracy, in degrees. The accuracy is defined as the
# precision with which the azimuth angle of a detection is reported.
float64 azimuth_accuracy

# Elevation angle resolution, in degrees. The resolution is defined as
# the minimum elevation angle between two objects which results in
# distinct detections.
float64 elevation_resolution

# Elevation angle accuracy, in degrees. The accuracy is defined as the
# precision with which the elevation angle of a detection is reported.
float64 elevation_accuracy
