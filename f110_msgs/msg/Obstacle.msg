int32 id

# Map-frame Cartesian representation. `has_cartesian` is explicit so consumers never interpret
# default-zero fields from an older or partially populated producer as a real obstacle at (0, 0).
bool has_cartesian
float64 x_center
float64 y_center
# Circular obstacle radius [m]. The detector encloses the complete Cartesian AABB:
# radius = 0.5 * hypot(x_max - x_min, y_max - y_min).
float64 radius
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 x_var
float64 y_var

# Frenet representation used by dynamic-opponent tracking and trajectory prediction.
float64 s_start
float64 s_end
float64 d_right           # defined as right bound of the obstacle
float64 d_left            # defined as left bound of the obstacle
bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go
float64 s_center
float64 d_center
float64 size
float64 vs
float64 vd
float64 s_var
float64 d_var
float64 vs_var
float64 vd_var
bool is_static
bool is_visible
# True when the dynamic opponent overlaps the ego driving corridor and its current or predicted
# longitudinal gap is small enough to interfere with ego driving.
bool is_interfering
