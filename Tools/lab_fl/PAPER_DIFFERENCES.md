# Differences with respect to the original paper

The paper (P. Albertos) designs two trajectory-tracking controllers for a quadrotor, LAB (Linear Algebra Based) and FL
(Feedback Linearization), and tests them on an ideal simulated plant. This repository runs them as native PX4 flight modes,
first in Gazebo (x500) and later on a real drone. This note lists what had to change, and why. Everything marked "measured"
comes from the Gazebo runs in `plots/comparison/` (and the plant-model study mentioned below); nothing has been flown on
hardware yet.

Video of LAB on the paper's second trajectory in Gazebo: `videos/lab_paper2_gazebo.mp4` (red dots: reference every 0.5 s,
green: start of the trajectory). Scripts used to record it: `videos/src/`.

## Summary

| # | Topic | Paper | Here | Parameter |
|---|---|---|---|---|
| 1 | Yaw in the translation dynamics | not present | rotate the horizontal acceleration by the heading | `TCC_HDG_COMP` (1) |
| 2 | Acceleration and jerk in FL | filtered velocity (0.2 s) | computed from the plant model | `FL_MODEL_DERIV` (1) |
| 3 | FL translation gains | poles on the imaginary axis (1.1 rad/s) | poles at -1..-2.5 by default; `fast2` preset at -3..-6 | `FL_K11`..`FL_K34` |
| 4 | FL yaw gains | 0.5 / 1.5 | 18 / 6 | `FL_K41`, `FL_K42` |
| 5 | Plant | ideal, no actuators | PX4 + Gazebo x500: estimator, allocation, motor lag | - |
| 6 | Sample time | 0.02 s (trajectory 1), 0.01 s (trajectory 2) | fixed `TCC_TS`, output held between samples | `TCC_TS` (0.02) |
| 7 | Limits | torque accelerations only | also thrust, tilt, normalized torque | `TCC_*` |
| 8 | Hover thrust | known | latched when entering the mode | - |
| 9 | Trajectories | Simulink | `traj_generator` (types 1 circle, 3 paper 1, 4 paper 2) | `TRJ_TYPE` |
| 10 | Safety | none | leave the mode to Hold | `TCC_ERR_MAX` etc. |

## 1. Heading compensation

The paper's translational model has no yaw: with x north, y east and z up, the horizontal acceleration is a function of
the thrust and of phi and theta only. On a real vehicle the thrust direction depends on the heading psi too, so the world
acceleration is `a_world = R(psi) a_h`, with `a_h` the paper's horizontal acceleration.

- With constant yaw (circle, yaw 0) the two coincide and nothing changes.
- With changing yaw (both trajectories of the paper) the uncompensated laws diverge: the roll and pitch commands end up
  pointing in the wrong direction while the heading turns.

Fix (`TCC_HDG_COMP`, default on):

- **LAB:** the desired velocity increment (dx, dy) is rotated by -psi before the phi_ref / theta_ref formulas.
- **FL:** the horizontal acceleration, jerk and snap are rotated with the first and second derivative of psi
  (`a_h'' = R^T v_world + psi_dot^2 a_h - 2 psi_dot J a_h' - psi_ddot J a_h`, with psi_ddot taken from the yaw law, which is
  computed first).

Covered by unit tests (`HeadingCompensationRotatesRollAndPitch`, `HeadingCompensationTracksWithRotatingHeading`).

## 2. FL: acceleration and jerk from the model

The paper estimates acceleration and jerk by filtering the measured velocity (filtered derivative, time constant 0.2 s).
On its ideal plant that is harmless. With any lag it destabilises the loop.

Measured in a plant-model study (paper gains, trajectory 1, five plants from ideal to "lag 100 ms and 1.5x gain"):

| Derivatives | Ideal plant | Plants with lag or delay |
|---|---|---|
| Filtered velocity (paper) | bounded, rms 0.77 m | unstable in all four (13 to 57 s) |
| From the model | rms 1.19 m | bounded in all four (0.82 to 1.17 m) |

The module therefore computes acceleration and jerk from the model by default (`FL_MODEL_DERIV 1`). `FL_MODEL_DERIV 0`
reproduces the paper (the filter then uses `max(FL_TAU, 10 TCC_TS)`). In Gazebo the paper gains with filtered derivatives
diverged on both trajectories.

## 3 and 4. FL gains

- The paper's gains put a complex pole pair exactly on the imaginary axis (neutrally stable on the ideal plant).
  Defaults here are real poles at -1, -1.5, -2, -2.5.
- `sitl.sh gains fl fast2` uses faster poles (-3..-6) with model derivatives. It gave the best results.
- Paper gains with its trajectory 1 or 2 are still available as presets `paper1` and `paper2`.
- The yaw loop uses `FL_K41 = 18`, `FL_K42 = 6` instead of 0.5 / 1.5. These are faster, so the yaw follows the turning reference closely (0.3 to 1.5 degrees RMS in Gazebo). They were not
  compared against the paper's yaw gains in isolation.

Also `FL_EXACT_FZ`: the paper's Simulink model wires the `u_dot` terms of `fz` slightly differently from the exact
derivative. Default 0 reproduces the paper; the effect is small.

## 5. Plant

The paper simulates an ideal rigid body that obeys the model. Here the loop includes the EKF2 estimate (noise and delay),
the PX4 control allocator and the motor dynamics of the Gazebo x500. This is why the gains of the paper do not carry over
unchanged (points 2 to 4). On the real drone expect more differences (mass, inertia, torque limits): the `TCC_*`
parameters must be calibrated (see below).

## 6. Sample time

Both controllers run at a fixed `TCC_TS` and hold their output between samples. LAB's `k1..k12` are fractions per sample,
so the gains of the paper only apply with the sample time they were designed for: 0.02 s for trajectory 1, 0.01 s for
trajectory 2. The presets `lab paper2` and `fl paper2` set `TCC_TS = 0.01`. Gains tuned for one trajectory do not transfer:
LAB with the trajectory 1 gains on trajectory 2 gave 1.1 m 3D RMSE (measured).

## 7 and 8. Limits and thrust

- `TCC_ALPHA_XY`, `TCC_ALPHA_Z` limit the commanded angular accelerations (paper: +-10 and +-1).
- Added: tilt limit (`TCC_TILT_MAX`), thrust limits (`TCC_THR_MIN/MAX`), and conversion of the torques to the normalized
  setpoints that PX4 expects (`TCC_TQ_MAX_XY`, `TCC_TQ_MAX_Z`, inertia `TCC_I*`, mass `TCC_MASS`).
- The modules publish `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` directly, so they replace the PX4 position,
  attitude and rate controllers while the mode is active.
- The hover thrust is latched when the mode is entered (the paper assumes it is known).
- Saturation of thrust, torque or tilt is reported in `traj_control_status` and logged (0 to 2.6 % of the time in the runs).

## 9. Trajectories

Implemented in `traj_generator` and checked against the files of the study:

- **Trajectory 1 (`TRJ_TYPE 3`):** 80 s, with a yaw ramp.
- **Trajectory 2 (`TRJ_TYPE 4`):** seven 5 s moves with a septic minimum-snap profile
  `s(t) = 35 t^4 - 84 t^5 + 70 t^6 - 20 t^7`: z +5, x +5, y +10, x -5, z -2, x +5, z -3 (north, east, up), with yaw alternating
  between pi and 0. Matches the reference file to 1e-13 (the last move differs by 8 mm). Peak velocity 4.4 m/s, peak yaw rate
  1.37 rad/s.
- Both start from the position where the mode is entered (relative trajectory).
- `TRJ_TYPE 1` is an extra circle with constant yaw, used for debugging.

## 10. Safety (not in the paper)

The mode leaves to Hold when the inputs are lost, the position error is above `TCC_ERR_MAX` for 0.3 s, the vehicle goes too
far (`TCC_DIST_MAX`) or too high (`TCC_ALT_MAX`), the tilt exceeds `TCC_TILT_ABORT`, or the law fails for 0.2 s. Details and
the pilot procedure in `SAFETY.md`.

## Results (Gazebo x500, 3D RMSE and yaw RMSE, best configuration per case)

| Trajectory | Controller | 3D RMSE [m] | Yaw RMSE [deg] |
|---|---|---|---|
| 1 | LAB, paper gains | 0.34 | 2.8 |
| 1 | FL, `fast2` | 0.058 | 0.41 |
| 2 | LAB, paper gains | 0.056 | 0.43 |
| 2 | FL, `fast2` | 0.19 | 1.4 |

Other configurations (measured): FL with the module defaults 0.34 m (trajectory 1) and 0.96 m (trajectory 2); FL with the
paper gains and filtered derivatives diverges on both; LAB with the module defaults was also flown (logs `paper*_lab_robust.ulg`, not included in this repository). The tilt stays between 9 and 20 degrees.

Caveats: one flight per configuration, one simulator, no wind or sensor-noise sweep. The gains were tuned by hand in
Gazebo, so the comparison between LAB and FL reflects how much tuning each received as well as the controllers themselves.

## Open points for the real drone

1. Calibrate `TCC_MASS`, `TCC_I*`, `TCC_TQ_MAX_XY/Z` and the hover thrust for the lab drone.
2. Rig tests without propellers, then with propellers on a rig, then free flight.
3. Choose the FL translation gains (defaults or `fast2`).
4. Re-check the heading convention on the real drone (magnetometer or GPS yaw) before flying a trajectory with yaw changes.
