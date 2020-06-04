# Lane Definition Message
# Contains information on a single lane marker

uint8 quality                 # Visual quality of lane marker
uint8 LANE_QUALITY_INVALID = 0
uint8 LANE_QUALITY_UNKNOWN = 1
uint8 LANE_QUALITY_NOT_AVAILABLE = 2
uint8 LANE_QUALITY_0 = 3
uint8 LANE_QUALITY_1 = 4
uint8 LANE_QUALITY_2 = 5
uint8 LANE_QUALITY_3 = 6
uint8 LANE_QUALITY_4 = 7
uint8 LANE_QUALITY_5 = 8
uint8 LANE_QUALITY_6 = 9
uint8 LANE_QUALITY_7 = 10
uint8 LANE_QUALITY_8 = 11
uint8 LANE_QUALITY_9 = 12
uint8 LANE_QUALITY_KIND_COUNT = 13

uint8 marker_kind             # Solid, dashed, ...
uint8 LANE_MARKER_INVALID = 0
uint8 LANE_MARKER_UNKNOWN = 1
uint8 LANE_MARKER_NOT_AVAILABLE = 2
uint8 LANE_MARKER_NONE = 3
uint8 LANE_MARKER_SOLID = 4
uint8 LANE_MARKER_DASHED = 5
uint8 LANE_MARKER_VIRTUAL = 6
uint8 LANE_MARKER_DOTS = 7
uint8 LANE_MARKER_ROAD_EDGE = 8
uint8 LANE_MARKER_DOUBLE_LINE = 9
uint8 LANE_MARKER_KIND_COUNT = 10

uint8 curve_model_kind        # Order of equation
uint8 LANE_CURVE_MODEL_INVALID = 0
uint8 LANE_CURVE_MODEL_UNKNOWN = 1
uint8 LANE_CURVE_MODEL_NOT_AVAILABLE = 2
uint8 LANE_CURVE_MODEL_LINEAR = 3
uint8 LANE_CURVE_MODEL_PARABOLIC = 4
uint8 LANE_CURVE_MODEL_3D = 5
uint8 LANE_CURVE_MODEL_KIND_COUNT = 6

float64 marker_offset         # Lateral distance from sensor to marker (m)
float64 heading_angle         # Angle of marker relative to sensor (rad)
float64 curvature             # Curvature of the lane marker at the camera (1/m)
float64 curvature_derivative  # Amount curvature changes as you move away from the camera (1/m^2)
float64 marker_width          # Width of the painted marker (not distance between lane markers) (m)
float64 view_range            # Physical view range of the lane mark (m)
