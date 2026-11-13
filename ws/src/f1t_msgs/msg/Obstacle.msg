# A single obstacle, described entirely in Frenet coordinates against the global
# raceline.
#
# Field names, types and order are copied verbatim from ForzaETH's
# f110_msgs/Obstacle so their planners port with no edits to the consuming code.
# Do not add fields here without a reason that outweighs that compatibility.
# In particular there is no is_opponent: ForzaETH removed it, and the line that
# read it survives commented out in collision_prediction.py. is_static carries
# that distinction instead.
#
# CONVENTIONS, which the publisher is responsible for upholding:
#
#   d > 0 is LEFT of the direction of travel, matching planning.frenet.FrenetConverter.
#
#   s_start < s_end always, even when the footprint straddles the start/finish line.
#   In that case s_end runs past the lap length rather than wrapping to near zero,
#   because downstream code sorts and takes min/max over these as plain numbers.
#   s_center is wrapped into [0, lap_length). Compare s values with
#   FrenetConverter.ds(), never by subtraction.
#
#   The *_var fields are the variance of the OBSERVATION, not the physical spread
#   of the object. A perfectly known obstacle still reports a small positive
#   variance; the Gaussian process consumers divide by it.

int32 id

float64 s_start
float64 s_end
float64 d_right              # the smaller (more negative) edge
float64 d_left               # the larger edge

bool is_actually_a_gap       # a hole in the wall rather than an object

float64 s_center
float64 d_center
float64 size                 # largest extent of the obstacle, metres

float64 vs                   # velocity along the reference line
float64 vd                   # velocity across it, positive to the left

float64 s_var
float64 d_var
float64 vs_var
float64 vd_var

bool is_static               # it will not move: cone, parked car, debris
bool is_visible              # seen this cycle, as opposed to remembered
