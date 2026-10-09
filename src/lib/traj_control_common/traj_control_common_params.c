/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file traj_control_common_params.c
 *
 * Parameters shared by the LAB and FL trajectory controllers.
 *
 * The model values below are REASONABLE DEFAULTS for a ~1.5 kg 450 mm quadrotor, NOT calibrated for the
 * lab drone. For the SIH simulator (sihsim_quadx) use TCC_MASS 1.0, TCC_IXX 0.025, TCC_IYY 0.025, TCC_IZZ 0.03,
 * TCC_TQ_MAX_XY 2.83 and TCC_TQ_MAX_Z 0.4. They must be replaced by the real mass, inertia and torque scale before flying (see the
 * calibration notes in the project README). Wrong values scale the control authority: inertia or torque
 * scale too high gives an aggressive, possibly unstable vehicle.
 */

/**
 * Vehicle mass
 *
 * @unit kg
 * @min 0.1
 * @max 50
 * @decimal 3
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_MASS, 1.5f);

/**
 * Inertia Ixx (roll)
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 5
 * @decimal 5
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_IXX, 0.02f);

/**
 * Inertia Iyy (pitch)
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 5
 * @decimal 5
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_IYY, 0.02f);

/**
 * Inertia Izz (yaw)
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 5
 * @decimal 5
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_IZZ, 0.04f);

/**
 * Roll / pitch torque at normalized torque 1
 *
 * Physical torque that the control allocator produces for a normalized roll or pitch torque setpoint of 1.
 * The allocator normalizes roll and pitch so that 1 moves every rotor output by 1/sqrt(2). For a quadrotor in X
 * configuration with a linear thrust model: 4 * L * T_rotor_max / sqrt(2), with L the arm length projected on the
 * roll / pitch axis and T_rotor_max the maximum thrust of one rotor. Default: 450 mm frame, 7.4 N per rotor.
 *
 * @unit Nm
 * @min 0.01
 * @max 100
 * @decimal 3
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_TQ_MAX_XY, 3.3f);

/**
 * Yaw torque at normalized torque 1
 *
 * Physical yaw torque that the control allocator produces for a normalized yaw torque setpoint of 1 (every rotor
 * output moves by 1). For a quadrotor: 4 * Q_rotor_max, with Q_rotor_max the reaction torque of one rotor at full
 * output. Default: 450 mm frame.
 *
 * @unit Nm
 * @min 0.001
 * @max 10
 * @decimal 3
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_TQ_MAX_Z, 0.47f);

/**
 * Minimum collective thrust
 *
 * Normalized thrust floor, keeps the motors spinning while the mode is active.
 *
 * @min 0
 * @max 0.5
 * @decimal 2
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_THR_MIN, 0.1f);

/**
 * Maximum collective thrust
 *
 * Normalized thrust ceiling, leaves headroom for the torque allocation.
 *
 * @min 0.3
 * @max 1
 * @decimal 2
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_THR_MAX, 0.9f);

/**
 * Roll / pitch angular acceleration limit
 *
 * Limit on phi_ddot and theta_ddot commanded by the controllers (the paper simulates +-10).
 *
 * @unit rad/s^2
 * @min 0.1
 * @max 100
 * @decimal 1
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_ALPHA_XY, 10.0f);

/**
 * Yaw angular acceleration limit
 *
 * Limit on psi_ddot commanded by the controllers (the paper simulates +-1).
 *
 * @unit rad/s^2
 * @min 0.1
 * @max 100
 * @decimal 1
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_ALPHA_Z, 1.0f);

/**
 * Maximum tilt angle
 *
 * Limit on the roll / pitch angles commanded by the controllers, and tilt above which the safety
 * monitoring leaves the mode.
 *
 * @unit deg
 * @min 5
 * @max 70
 * @decimal 0
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_TILT_MAX, 35.0f);

/**
 * Controller sample time
 *
 * Both LAB and FL run at this fixed rate and hold their output between samples. The gains of the
 * discrete LAB law were designed for 0.02 s (a different value needs gains re-tuned).
 *
 * @unit s
 * @min 0.004
 * @max 0.05
 * @decimal 3
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_TS, 0.02f);

/**
 * Reference timeout
 *
 * If trajectory_reference is older than this the controllers leave the mode.
 *
 * @unit s
 * @min 0.05
 * @max 2
 * @decimal 2
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_REF_TOUT, 0.2f);

/**
 * State estimate timeout
 *
 * If vehicle_local_position or the attitude is older than this the controllers leave the mode.
 *
 * @unit s
 * @min 0.05
 * @max 2
 * @decimal 2
 * @group Trajectory Control
 */
PARAM_DEFINE_FLOAT(TCC_EST_TOUT, 0.2f);

/**
 * Heading compensation
 *
 * The laws of the study assume that the vehicle heads North (its translation model does not contain the yaw angle).
 * 0: apply them as they are (valid only while the heading is constant and North).
 * 1: apply them in the frame aligned with the current heading: the desired horizontal acceleration is rotated by the
 * heading before it is turned into roll and pitch (exact for the FL law, including the terms of the rotation).
 * With a heading of 0 both settings are identical.
 *
 * @value 0 Off (as in the paper)
 * @value 1 On
 * @group Trajectory Control
 */
PARAM_DEFINE_INT32(TCC_HDG_COMP, 1);
