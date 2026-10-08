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
 * @file lab_control_params.c
 *
 * Gains k_1 .. k_12 of the LAB controller (build_controller_gains.m of the study, first trajectory,
 * Ts = 0.02 s): [x, vx, y, vy, z, vz, phi, phi_dot, theta, theta_dot, psi, psi_dot].
 * Each gain is the fraction of the error that is NOT closed in one sample (0 closes it in one step).
 * The second trajectory of the study (Ts = 0.01 s) uses k = 0.99 0.89 0.99 0.89 0.99 0.79 0.85 0.81 0.85 0.81 0.85 0.95.
 */

/**
 * LAB gain k_1 (x position)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K1, 0.99f);

/**
 * LAB gain k_2 (x velocity)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K2, 0.97f);

/**
 * LAB gain k_3 (y position)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K3, 0.99f);

/**
 * LAB gain k_4 (y velocity)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K4, 0.98f);

/**
 * LAB gain k_5 (z position)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K5, 0.99f);

/**
 * LAB gain k_6 (z velocity)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K6, 0.79f);

/**
 * LAB gain k_7 (roll angle)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K7, 0.96f);

/**
 * LAB gain k_8 (roll rate)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K8, 0.91f);

/**
 * LAB gain k_9 (pitch angle)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K9, 0.96f);

/**
 * LAB gain k_10 (pitch rate)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K10, 0.92f);

/**
 * LAB gain k_11 (yaw angle)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K11, 0.99f);

/**
 * LAB gain k_12 (yaw rate)
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group LAB Controller
 */
PARAM_DEFINE_FLOAT(LAB_K12, 0.95f);
