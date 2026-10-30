# Gazebo model of the lab drone (ZMR250)

A Gazebo (gz) model of the lab drone built from its CAD, its weights and the thrust-bench data of its motor, to fly the LAB
and FL modes in simulation on the real vehicle's dynamics instead of the x500. The model is **generated**: change a mass, a
position or the motor data in `zmr250.toml` and run `make_model.py`. The generated files are committed, so nothing has to be
generated to simulate.

| File | What |
|---|---|
| `zmr250.toml` | masses, positions and motor settings: the one file to edit (each mass has a `status`: measured, datasheet, estimate) |
| `make_model.py` | writes `models/zmr250/model.sdf`, the SITL airframe `ROMFS/.../airframes/4050_gz_zmr250`, `params/zmr250.params` and `mass_report.md` |
| `identify_motor.py` | fits the motor and propeller model from a thrust-bench CSV (RCbenchmark): kT, kQ, rotor speed against output, `THR_MDL_FAC` |
| `convert_cad.py` | STEP/STL of the CAD to the meshes of the model and the exact properties of the frame (needs FreeCAD) |
| `data/` | bench CSV of the 1806 motor with the 4045 propeller at 4S, and the exact CAD properties of the frame |
| `mass_report.md` | mass, centre of mass, inertia, thrust margins and the `TCC_*` values of the current configuration |
| `convert_beacon.py` | STL of the Marvelmind Super-Beacon (vendor zip) to the mesh of the model |
| `sweep_motor_lag.sh` | flies the second trajectory of the paper for several rotor time constants |
| `../videos/make_video.sh` | records a video of a flight (wide view + chase camera) |

## Fly it

    # once: build the Gazebo SITL (see the header of ../sitl.sh) after pulling, so the airframe is in the build
    VEHICLE=zmr250 Tools/lab_fl/sitl.sh start gz paper2      # the lab drone instead of the x500
    Tools/lab_fl/sitl.sh gains lab paper2                     # or: fl fast2
    Tools/lab_fl/sitl.sh demo lab
    VEHICLE=zmr250 Tools/lab_fl/experiment.sh paper2 fl fast2   # automatic flight, log in ~/px4_sitl_labfl/runs/zmr250/

`sitl.sh` links the model into the Gazebo models directory (a git submodule: the link is kept out of its status). The airframe
sets the geometry of the rotors relative to the centre of mass, the mapping of the actuator output to the rotor speed
(`SIM_GZ_EC_*`), the thrust curve (`THR_MDL_FAC`), the hover thrust and the `TCC_*` mass, inertia and torque scales.

## What it is built from

- **Geometry:** the frame, motors, Pixhawk and printed legs are the CAD of the TFE (`TFE_Mario_Bleda_Simulacion_Drone`),
  converted to STL. The flight direction is the long axis of the frame plates (Y of the CAD); the motors are at
  (+-78.3, +-98.8) mm, 252 mm between opposite motors. The legs are one bar at the front and one at the rear.
- **Mass and inertia:** the frame's mass is spread over the exact volume of its CAD; the Pixhawk, battery, wiring and beacon are
  boxes; the ESC are point masses; the legs are the STL volume times the ASA density and the infill factor. Rotor links carry the
  propeller and 40 % of each motor.
- **Motor:** thrust = kT w^2 and torque = kQ w^2 with kT 2.20e-7 N/(rad/s)^2 and kQ 3.09e-9 N m/(rad/s)^2 from the bench
  (the fit reproduces the one of the TFE). The rotor speed is a linear function of the PX4 output (Gazebo plugin limitation),
  fitted over the output range above 0.4, where the drone flies.

## What is assumed (check before trusting a result)

| Assumption | Why it matters |
|---|---|
| **Rotor speed lag (`motor.time_constant`).** The TFE used 30 ms as a setting, not as an identification: the bench sweep is quasi-static. | LAB with the paper gains is unstable at 30 ms and flies at 12.5 ms (see the results below). Measure it on the bench with a PWM step. |
| Battery under the frame, centred; Pixhawk centred on the top plate; beacon on top | Centre of mass and inertia |
| ESC (20 g), wiring (28 g), beacon (35 g), motors (20 g each) are estimates | Total mass and inertia |
| Battery voltage constant (the bench ran at 16.8 to 15.9 V) | Thrust falls with the state of charge: at 14.8 V roughly a fifth of the thrust is lost (estimate: thrust scales with the square of the voltage) |
| Propeller CAD is not a 4045 (317 mm blade): used scaled, for the look only | Physics comes from kT, not from the mesh |
| No drag, ground effect or propeller-frame interaction | The simulation is optimistic at speed |
| IMU at the centre of mass | PX4 assumes it; the real Pixhawk sits about 4 cm above it (`EKF2_IMU_POS_Z`) |
| Standard PX4 X numbering and spin directions | Check the motor outputs on the real drone |

## Real drone

`params/zmr250.params` has the same PX4 parameters (rotor geometry, thrust model, hover thrust, `TCC_*`) as a QGroundControl
file. The PWM range of the ESC (`PWM_MAIN_MIN/MAX`) must be set to the bench range (1030 to 1900 us). Replace each estimated
mass of `zmr250.toml` with the weighed one first, then run `make_model.py`.

## Results (Gazebo, lab drone, second trajectory of the paper)

Mass 615 g at the time of the sweep (Pixhawk on the top plate, 35 g beacon); the current model is 637 g (see `mass_report.md`).
3D RMSE of the position and RMS yaw error while the mode is active; "left the mode" means the safety monitor sent the vehicle
to Hold. One flight per cell.

| Rotor lag | LAB, paper gains (`lab paper2`) | FL, `fast2` |
|---|---|---|
| 12.5 ms | 0.056 m, yaw 0.5 deg, full trajectory | 0.198 m, yaw 2.0 deg, full trajectory |
| 20 ms | left the mode after 1.9 s | 0.199 m, yaw 2.2 deg |
| 30 ms | left the mode after 0.6 s | 0.203 m, yaw 2.1 deg |
| 50 ms | left the mode after 0.5 s | 0.198 m, yaw 2.1 deg |

- **FL does not depend on the rotor lag** in this range. It saturates the thrust 18 % of the time (thrust-to-weight is only
  about 1.4) and tilts up to 19 degrees.
- **LAB with the gains of the paper only works with a lag of about 12 ms or less.** Those gains are an almost deadbeat design
  for an ideal torque actuator. LAB with the module's default gains (`lab robust`, 30 ms) also fails: it left the mode after
  11.8 s (0.91 m RMSE). LAB needs gains designed for the real lag before it flies this drone: an open task.
- **The rotor lag of the real motor and propeller is unknown.** Measure it on the bench with a PWM step before trusting any LAB
  result. The default of the model is 30 ms, the value used in the TFE.

A new motor or propeller: put its bench CSV in `data/`, point `motor.bench_csv` to it, set `pwm_min` and `pwm_max`, and run
`make_model.py`. Check that the thrust-to-weight ratio of `mass_report.md` is at least 1.5.
