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
 * @file TrajControlCommon.hpp
 *
 * Everything the LAB and FL controllers share besides their control law: the vehicle model parameters
 * (TCC_*), the common limits, the sample time, the reference / state timeouts, publication of the thrust and
 * torque setpoints, and the traj_control_status message. Keeps both controllers from diverging on anything that
 * is not the control law itself.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/traj_control_status.h>
#include <uORB/topics/trajectory_reference.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "TrajControlMath.hpp"

namespace traj_control
{

/**
 * Inputs shared by LAB and FL, in the paper's frame (x North, y East, z Up; see TrajControlMath.hpp).
 * Both controllers read the state from EKF2 through exactly the same path.
 */
struct Inputs {
	matrix::Vector3f position{};       // [m]
	matrix::Vector3f velocity{};       // [m/s]
	matrix::Vector3f euler{};          // phi, theta, psi [rad]
	matrix::Vector3f euler_rate{};     // [rad/s]
	matrix::Vector3f ref_position{};   // reference at this sample
	matrix::Vector3f ref_velocity{};
	matrix::Vector3f ref_position_next{}; // reference one sample ahead (the discrete laws use x_ref(i+1))
	float ref_yaw{0.f};
	float ref_yaw_next{0.f};
	float ref_yaw_rate{0.f};
	hrt_abstime timestamp_sample{0};
	trajectory_reference_s reference{};  // as received (NED), for the status message
	vehicle_local_position_s local_position{};
};

class TrajControlCommon : public ModuleParams
{
public:
	explicit TrajControlCommon(ModuleParams *parent);
	~TrajControlCommon() = default;

	/** Reload TCC_* and MPC_THR_HOVER. Called automatically through the ModuleParams parent chain. */
	void updateParams() override;

	const VehicleModel &model() const { return _model; }

	/**
	 * Latch the hover thrust for this run of the mode: the live estimate of mc_hover_thrust_estimator (converged while
	 * the vehicle was flying in Position or Hold before entering) if valid, otherwise MPC_THR_HOVER. It is not updated
	 * while the mode is active, so the estimator cannot interact with the controller. Call on every entry into the mode.
	 */
	void latchHoverThrust();
	const Limits &limits() const { return _limits; }

	/** Controller sample time Ts [s] (TCC_TS). Both controllers run at the same rate. */
	float samplePeriod() const { return _param_tcc_ts.get(); }
	hrt_abstime referenceTimeout() const { return static_cast<hrt_abstime>(_param_tcc_ref_tout.get() * 1e6f); }
	hrt_abstime estimateTimeout() const { return static_cast<hrt_abstime>(_param_tcc_est_tout.get() * 1e6f); }

	/**
	 * Read the latest state estimate and reference. Returns false (and leaves `in` unspecified) if any input is
	 * missing, older than TCC_EST_TOUT / TCC_REF_TOUT, or flagged invalid (position, heading or reference).
	 */
	bool readInputs(hrt_abstime now, Inputs &in);

	/** Convert the paper's inputs into normalized setpoints (see traj_control::toSetpoints). */
	Setpoints compute(const matrix::Vector3f &euler, const matrix::Vector3f &euler_rate, const Command &command) const
	{
		return toSetpoints(_model, _limits, euler, euler_rate, command);
	}

	/** Publish vehicle_thrust_setpoint and vehicle_torque_setpoint. Only the active controller may call this. */
	void publishSetpoints(const Setpoints &setpoints, hrt_abstime timestamp, hrt_abstime timestamp_sample);

	/**
	 * Publish a neutral setpoint (hover thrust, zero torque). Used while the mode is active but the inputs are
	 * missing or stale, until the safety logic takes the vehicle out of the mode. It keeps the setpoints fresh
	 * instead of letting the allocator hold the last control output.
	 */
	void publishNeutral(hrt_abstime timestamp);

	/** Restart the per-run saturation counter (call on every entry into the mode). */
	void resetSaturationCount() { _saturation_count = 0; }

	/**
	 * Publish traj_control_status. Shadow controllers (active = false) also call this so their output can be
	 * logged and compared. q_d may be nullptr (FL has no explicit attitude reference: it is logged as NaN).
	 */
	void publishStatus(uint8_t controller, bool active, hrt_abstime timestamp, const trajectory_reference_s &ref,
			   const vehicle_local_position_s &lpos, float yaw, const Setpoints &setpoints,
			   bool tilt_saturated, const float *q_d);

private:
	VehicleModel _model{};
	Limits _limits{};
	uint32_t _saturation_count{0};
	bool _hover_latched{false};
	float _hover_thrust_latched{0.5f};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _reference_sub{ORB_ID(trajectory_reference)};

	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<traj_control_status_s> _status_pub{ORB_ID(traj_control_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TCC_MASS>) _param_tcc_mass,
		(ParamFloat<px4::params::TCC_IXX>) _param_tcc_ixx,
		(ParamFloat<px4::params::TCC_IYY>) _param_tcc_iyy,
		(ParamFloat<px4::params::TCC_IZZ>) _param_tcc_izz,
		(ParamFloat<px4::params::TCC_TQ_MAX_XY>) _param_tcc_tq_max_xy,
		(ParamFloat<px4::params::TCC_TQ_MAX_Z>) _param_tcc_tq_max_z,
		(ParamFloat<px4::params::TCC_THR_MIN>) _param_tcc_thr_min,
		(ParamFloat<px4::params::TCC_THR_MAX>) _param_tcc_thr_max,
		(ParamFloat<px4::params::TCC_ALPHA_XY>) _param_tcc_alpha_xy,
		(ParamFloat<px4::params::TCC_ALPHA_Z>) _param_tcc_alpha_z,
		(ParamFloat<px4::params::TCC_TILT_MAX>) _param_tcc_tilt_max,
		(ParamFloat<px4::params::TCC_TS>) _param_tcc_ts,
		(ParamFloat<px4::params::TCC_REF_TOUT>) _param_tcc_ref_tout,
		(ParamFloat<px4::params::TCC_EST_TOUT>) _param_tcc_est_tout,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover
	)
};

} // namespace traj_control
