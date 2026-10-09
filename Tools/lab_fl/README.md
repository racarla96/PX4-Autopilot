# LAB and FL flight modes

Two trajectory-tracking controllers from the study by Pedro Albertos, implemented as native PX4 v1.17 flight modes:

- **LAB** (Linear Algebra Based): discrete law, needs the reference at samples i and i+1.
- **FL** (Feedback Linearization): dynamic extension with pole-placement gains.

Both publish `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` directly. Branch: `feature/lab-fl-modes`.

| Piece | Where |
|---|---|
| Reference generator (circle, paper 1, paper 2) | `src/modules/traj_generator` |
| LAB law / module | `src/modules/lab_control` |
| FL law / module | `src/modules/fl_control` |
| Shared code (inputs, hover thrust, heading compensation, safety monitor) | `src/lib/traj_control_common` |
| Tools | `Tools/lab_fl/` |

Modes: `LAB` is nav_state 7, `FL` is nav_state 9. Hold is nav_state 4.

## Build

Everything runs in Docker image `px4io/px4-dev:v1.17.0`:

    docker run --rm -u $(id -u):$(id -g) -v "$PWD":/src -w /src -e HOME=/tmp px4io/px4-dev:v1.17.0 make px4_fmu-v3_default

Flash with QGroundControl (custom firmware) or `make px4_fmu-v3_default upload`.
Flash use on fmu-v3 is 98.68 % (limit 2,080,768 B, about 27 KB free). Only if a build no longer fits, remove modules you do not use (fixed wing, VTOL, UUV) in
`boards/px4/fmu-v3/default.px4board`. Check the size with `arm-none-eabi-size`.

Unit tests (laws, trajectories, safety monitor):

    cd build/px4_sitl_test && ninja unit-LabControlLawTest unit-FlControlLawTest unit-TrajectoryProfileTest unit-SafetyMonitorTest && ./unit-<name>

## Simulation (SITL)

    Tools/lab_fl/sitl.sh start gz paper2     # Gazebo x500 on the host (also: sih, in Docker)
    Tools/lab_fl/sitl.sh gains fl fast2      # gains preset
    Tools/lab_fl/sitl.sh demo fl             # arm, take off, enter the mode
    Tools/lab_fl/sitl.sh hold | land | stop

Gazebo build: `make px4_sitl_default BUILD_DIR_SUFFIX=_gz` (needs Gazebo Harmonic). `start gz` always begins with default parameters.

Automated experiment and analysis:

    Tools/lab_fl/experiment.sh paper2 fl fast2          # writes ~/px4_sitl_labfl/runs/paper2_fl_fast2.ulg
    python3 Tools/lab_fl/analyze.py --out out ~/px4_sitl_labfl/runs/*.ulg    # metrics.csv + plots (needs pyulog, numpy, matplotlib)

## Gains presets (`sitl.sh gains <lab|fl> <preset>`)

- `robust`: module defaults.
- `paper1`, `paper2`: the study's gains for its trajectory 1 or 2. FL with the paper's filtered derivatives diverges in simulation.
- `fast2` (FL): poles at -3..-6, yaw gains 18/6, model derivatives (`FL_MODEL_DERIV 1`).

## Results in Gazebo x500 (best configuration per case)

| Trajectory | Controller | 3D RMSE [m] | Yaw RMSE [deg] |
|---|---|---|---|
| Paper 1 | LAB, paper1 gains | 0.34 | 2.8 |
| Paper 1 | FL, fast2 | 0.058 | 0.41 |
| Paper 2 | LAB, paper2 gains | 0.056 | 0.43 |
| Paper 2 | FL, fast2 | 0.19 | 1.4 |

Plots and all metrics: `Tools/lab_fl/plots/comparison/`. LAB gains tuned for one trajectory do not transfer (paper 1 gains on paper 2: 1.1 m).

## Important design points

- **Heading compensation** (`TCC_HDG_COMP`, default 1): the paper's model has no yaw in the translation. The real vehicle does, so the laws rotate by the current heading. Without it, any trajectory with changing yaw diverges.
- **Trajectory parameters**: `TRJ_TYPE` 1 circle, 3 paper 1, 4 paper 2.
- **Safety**: see `SAFETY.md`. The mode returns to Hold on lost inputs, large error, distance, height, tilt or controller failure (`TCC_*` parameters).

## Before flying the real drone

Not done yet; the results above are simulation only.

1. Set the vehicle parameters (`TCC_*`: mass, inertia, maximum torques, hover thrust) for the lab drone.
2. Test on a rig without propellers, then with propellers on a rig, then free flight.
3. Check the safety limits against the lab space and keep the pilot ready to switch to a manual mode.
4. Decide the FL translation gains: `robust` (defaults) or `fast2` (more aggressive, only tested in simulation).
