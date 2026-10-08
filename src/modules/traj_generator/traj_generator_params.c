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
 * @file traj_generator_params.c
 *
 * Parameters of the reference trajectory shared by the LAB and FL flight modes.
 * Values are latched when the trajectory starts, so changing them in flight only
 * takes effect at the next entry into LAB or FL.
 */

/**
 * Trajectory type
 *
 * @value 0 Hover (hold the start point)
 * @value 1 Circle
 * @value 2 Figure-8
 * @value 3 Paper (LAB vs FL study: 2 m circle at 0.3 rad/s, 2 m climb, 2 m descent, 80 s; ignores the other TRJ_* parameters)
 * @group Trajectory Generator
 */
PARAM_DEFINE_INT32(TRJ_TYPE, 3);

/**
 * Trajectory amplitude
 *
 * Circle radius, or half-width of the figure-8.
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 2
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_AMP, 1.0f);

/**
 * Trajectory frequency
 *
 * Cruise rate of the path: one lap takes 1 / TRJ_FREQ seconds.
 *
 * @unit Hz
 * @min 0.01
 * @max 1
 * @decimal 3
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_FREQ, 0.1f);

/**
 * Trajectory altitude
 *
 * Altitude above the local origin (up is positive) reached smoothly after entering
 * the mode. 0 keeps the altitude the vehicle had when the trajectory started.
 *
 * @unit m
 * @min 0
 * @max 50
 * @decimal 1
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_ALT, 0.0f);

/**
 * Altitude change time
 *
 * Duration of the smooth transition to TRJ_ALT.
 *
 * @unit s
 * @min 1
 * @max 60
 * @decimal 1
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_ALT_T, 5.0f);

/**
 * Trajectory duration
 *
 * 0 runs forever. With a finite duration the trajectory ends at rest and flags finished.
 *
 * @unit s
 * @min 0
 * @max 3600
 * @decimal 1
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_DURATION, 60.0f);

/**
 * Speed ramp time
 *
 * Time to accelerate from rest to cruise speed (and to stop at the end of a finite
 * trajectory). 0 disables the ramp, which makes the reference start with a velocity step.
 *
 * @unit s
 * @min 0
 * @max 60
 * @decimal 1
 * @group Trajectory Generator
 */
PARAM_DEFINE_FLOAT(TRJ_RAMP, 5.0f);
