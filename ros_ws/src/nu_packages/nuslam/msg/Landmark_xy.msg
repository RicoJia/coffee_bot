#custom message type for a cylindrical landmark
#position_covariance: assume diagonal standard deviation, [range_stddev, bearing_stddev]

float64 last_update
float64 x
float64 y
float64 radius
float64[2] position_stddev
uint8 landmark_id
