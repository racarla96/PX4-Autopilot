# Safety of the LAB and FL modes

The two modes bypass PX4's attitude and rate loops (they publish thrust and torque setpoints directly), so a fault in the
state estimate, in the reference or in the law is not caught by the usual position controller. What protects the vehicle:

## What leaves the mode by itself (Hold, with a message in QGroundControl)
Implemented in `src/lib/traj_control_common/SafetyMonitor.*`, checked at every controller sample (`TCC_*` parameters):

| Condition | Parameter | Default |
|---|---|---|
| State estimate or reference missing, stale or invalid for longer than the timeout (a neutral setpoint, hover thrust and zero torque, is sent meanwhile) | `TCC_FAIL_TOUT` | 0.5 s |
| Position error above the limit for more than 0.3 s | `TCC_ERR_MAX` | 3 m |
| Horizontal distance from the point where the mode was entered | `TCC_DIST_MAX` | 20 m |
| Height above the point where the mode was entered | `TCC_ALT_MAX` | 12 m |
| Tilt of the vehicle | `TCC_TILT_ABORT` | 70 deg |
| The law fails (singular decoupling matrix, non finite output) for more than 0.2 s | | |

`0` disables a check. The request is repeated every 0.5 s until the mode is left. The thrust and attitude limits of the
laws themselves are `TCC_THR_MIN/MAX` and `TCC_TILT_MAX`.

## What PX4 does anyway
The modes register their requirements with the commander (`mode_requirements.cpp`: manual control, local position, attitude),
so the standard failsafes apply: loss of the RC link, loss of the position estimate, battery, geofence, arming checks.
Verified in SITL: stopping the manual control source while in LAB makes the commander take over (Hold for 5 s, then RTL).

## What the pilot has to do
- Fly in Position or Hold first (the hover thrust is taken from the estimator when the mode is entered).
- Keep a mode switch on the transmitter for Hold / Position at all times: any other mode can be selected from any mode,
  and the modes are left at once.
- LAB and FL can be selected one after the other; each entry restarts the trajectory from the current position and heading.
- First flights: tethered or on the gimbal rig without propellers, then with propellers at a low `TRJ_AMP`, then the
  trajectories of the study (the second reaches 4.4 m/s and 1.4 rad/s of yaw: scale it up in steps).

## Verified in Gazebo SITL (x500)
Reference lost (`traj_generator stop`) -> Hold; position error limit -> Hold; distance limit -> Hold; manual Hold; Hold -> FL;
FL -> LAB (the trajectory restarts); RC loss -> failsafe. Unit tests: `make tests TESTFILTER=SafetyMonitor`.
