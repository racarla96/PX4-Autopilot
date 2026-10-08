# Common status of the LAB and FL trajectory controllers
# Same format for both controllers so that logs can be compared directly.
# Published by lab_control and fl_control, also while running in shadow (inactive) mode.

uint64 timestamp		# time since system start (microseconds)

uint8 CONTROLLER_NONE = 0
uint8 CONTROLLER_LAB = 1
uint8 CONTROLLER_FL = 2

uint8 controller		# which controller produced this message
bool active			# true: output applied to vehicle_attitude_setpoint, false: shadow mode (computed and logged only)

float32[3] position_ref		# reference position in NED (m)
float32[3] velocity_ref		# reference velocity in NED (m/s)
float32[3] position		# estimated position in NED (m), from EKF2
float32[3] velocity		# estimated velocity in NED (m/s), from EKF2

float32[3] position_error	# position_ref - position (m)
float32[3] velocity_error	# velocity_ref - velocity (m/s)
float32 yaw_error		# yaw_ref - yaw (rad), wrapped to -PI..+PI

float32 thrust			# normalized collective thrust output [0,1]
float32[4] q_d			# desired attitude quaternion sent to the attitude controller

bool thrust_saturated		# thrust limited by min/max
bool tilt_saturated		# tilt angle limited by the configured maximum
uint32 saturation_count		# number of cycles with any saturation since the mode was entered
