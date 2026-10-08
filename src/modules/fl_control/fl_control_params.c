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
 * @file fl_control_params.c
 *
 * Pole placement gains of the FL controller (build_controller_gains.m of the study, first trajectory, Ts = 0.02 s).
 * Channel x: v1 = k11 e + k12 e_dot - k13 x_ddot - k14 x_dddot; channels y (k2*) and z (k3*) are the same;
 * yaw: v4 = k41 e_psi + k42 e_psi_dot.
 * Second trajectory of the study (Ts = 0.01 s): K1 = 0 49.375 6.25 7.9, K2 = 0 50 6.25 8, K3 = 0 58.32 7.29 8, K4 = 18 6.
 */

/**
 * FL gain k11 (channel x, position error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K11, 1.452f);

/**
 * FL gain k12 (channel x, velocity error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K12, 3.509f);

/**
 * FL gain k13 (channel x, estimated acceleration)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K13, 2.41f);

/**
 * FL gain k14 (channel x, estimated jerk)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K14, 2.9f);

/**
 * FL gain k21 (channel y, position error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K21, 1.26f);

/**
 * FL gain k22 (channel y, velocity error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K22, 2.5f);

/**
 * FL gain k23 (channel y, estimated acceleration)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K23, 2.26f);

/**
 * FL gain k24 (channel y, estimated jerk)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K24, 2.5f);

/**
 * FL gain k31 (channel z, position error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K31, 1.6698f);

/**
 * FL gain k32 (channel z, velocity error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K32, 3.509f);

/**
 * FL gain k33 (channel z, estimated acceleration)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K33, 2.59f);

/**
 * FL gain k34 (channel z, estimated jerk)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K34, 2.9f);

/**
 * FL gain k41 (yaw channel, yaw error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K41, 0.5f);

/**
 * FL gain k42 (yaw channel, yaw rate error)
 *
 * @min 0
 * @max 1000
 * @decimal 4
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_K42, 1.5f);

/**
 * Derivative filter time constant
 *
 * Time constant of the filters that estimate acceleration and jerk from the velocity. As in the Filtered Derivative
 * block of the study, the effective value is max(FL_TAU, 10 * TCC_TS), so 0.2 s with the default sample time.
 *
 * @unit s
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group FL Controller
 */
PARAM_DEFINE_FLOAT(FL_TAU, 0.01f);

/**
 * Use the exact fz drift term
 *
 * The paper's Simulink model wires the u_dot terms of fz differently from the exact derivative of the vertical
 * dynamics. 0 reproduces the paper (default), 1 uses the exact derivative. The effect is small.
 *
 * @boolean
 * @group FL Controller
 */
PARAM_DEFINE_INT32(FL_EXACT_FZ, 0);
