# This represents a detected or tracked object with reference coordinate frame and timestamp.

std_msgs/Header header

# The id of the object (presumably from the detecting sensor).
uint32 id

# A Detected object is one which has been seen in at least one scan/frame of a sensor.
# A Tracked object is one which has been correlated over multiple scans/frames of a sensor.
# An object which is detected can only be assumed to have valid pose and shape properties.
# An object which is tracked should also be assumed to have valid twist and accel properties.
uint8 detection_level
uint8 OBJECT_DETECTED=0
uint8 OBJECT_TRACKED=1

# A Classified object is one which has been identified as a certain object type.
# Classified objects should have valid classification, classification_certainty, and classification_age properties.
bool object_classified

# The detected position and orientation of the object.
geometry_msgs/Pose pose

# The detected linear and angular velocities of the object.
geometry_msgs/Twist twist

# The detected linear and angular accelerations of the object.
geometry_msgs/Accel accel

# (OPTIONAL) The polygon defining the detection points at the outer edges of the object.
geometry_msgs/Polygon polygon

# A shape conforming to the outer bounding edges of the object.
shape_msgs/SolidPrimitive shape

# The type of classification given to this object.
uint8 classification
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_UNKNOWN_SMALL=1
uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2
uint8 CLASSIFICATION_UNKNOWN_BIG=3
uint8 CLASSIFICATION_PEDESTRIAN=4
uint8 CLASSIFICATION_BIKE=5
uint8 CLASSIFICATION_CAR=6
uint8 CLASSIFICATION_TRUCK=7
uint8 CLASSIFICATION_MOTORCYCLE=8
uint8 CLASSIFICATION_OTHER_VEHICLE=9
uint8 CLASSIFICATION_BARRIER=10
uint8 CLASSIFICATION_SIGN=11

# The certainty of the classification from the originating sensor.
# Higher value indicates greater certainty (MAX=255).
# It is recommended that a native sensor value be scaled to 0-255 for interoperability.
uint8 classification_certainty

# The number of scans/frames from the sensor that this object has been classified as the current classification.
uint32 classification_age
