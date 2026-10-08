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
 * @file LabControl.hpp
 *
 * LAB trajectory-tracking flight mode. While nav_state is LAB it reads the state from EKF2 and the reference from
 * traj_generator, runs the discrete LAB law at TCC_TS, and publishes vehicle_thrust_setpoint and
 * vehicle_torque_setpoint. It publishes nothing in any other mode.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/traj_control_common/TrajControlCommon.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

#include "LabControlLaw.hpp"

class LabControl : public ModuleBase<LabControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LabControl();
	~LabControl() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;
	void updateParams() override;
	void schedule();

	lab_control::Gains gains() const;

	traj_control::TrajControlCommon _common{this};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uint8_t _nav_state{vehicle_status_s::NAVIGATION_STATE_MANUAL};
	bool _was_active{false};
	uint32_t _interval_us{20000};
	hrt_abstime _last_input_failure{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LAB_K1>) _param_k1,
		(ParamFloat<px4::params::LAB_K2>) _param_k2,
		(ParamFloat<px4::params::LAB_K3>) _param_k3,
		(ParamFloat<px4::params::LAB_K4>) _param_k4,
		(ParamFloat<px4::params::LAB_K5>) _param_k5,
		(ParamFloat<px4::params::LAB_K6>) _param_k6,
		(ParamFloat<px4::params::LAB_K7>) _param_k7,
		(ParamFloat<px4::params::LAB_K8>) _param_k8,
		(ParamFloat<px4::params::LAB_K9>) _param_k9,
		(ParamFloat<px4::params::LAB_K10>) _param_k10,
		(ParamFloat<px4::params::LAB_K11>) _param_k11,
		(ParamFloat<px4::params::LAB_K12>) _param_k12
	)
};
