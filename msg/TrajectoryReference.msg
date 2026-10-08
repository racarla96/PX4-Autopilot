# Reference trajectory for the LAB and FL trajectory-tracking flight modes
# Published by traj_generator. Local NED frame, same as vehicle_local_position.
# The trajectory clock restarts on every transition into the LAB or FL mode.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_start		# trajectory clock origin, restarted on each mode transition (microseconds)

float32 t			# time since timestamp_start (seconds)

float32[3] position		# reference position in NED (meters)
float32[3] velocity		# reference velocity in NED (m/s)
float32[3] acceleration		# reference acceleration in NED (m/s^2)
float32[3] jerk			# reference jerk in NED (m/s^3), required by FL

float32 yaw			# reference yaw (rad), NED, -PI..+PI
float32 yaw_rate		# reference yaw rate (rad/s)

bool valid			# true while the reference is being generated and can be tracked
bool finished			# true once the configured trajectory duration has elapsed
