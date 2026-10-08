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
 * Unit tests for the math shared by the LAB and FL controllers.
 * Run only these using: make tests TESTFILTER=TrajControlMath
 */

#include <gtest/gtest.h>
#include <math.h>

#include "TrajControlMath.hpp"

using namespace traj_control;
using matrix::Vector3f;

namespace
{

Vector3f eulerAt(float t)
{
	return Vector3f(0.30f * sinf(0.9f * t), -0.25f * cosf(0.7f * t), 0.5f * t);
}

Vector3f eulerRateAt(float t)
{
	return Vector3f(0.30f * 0.9f * cosf(0.9f * t), 0.25f * 0.7f * sinf(0.7f * t), 0.5f);
}

Vector3f eulerAccelAt(float t)
{
	return Vector3f(-0.30f * 0.81f * sinf(0.9f * t), 0.25f * 0.49f * cosf(0.7f * t), 0.f);
}

} // namespace

TEST(TrajControlMath, FrameConversionIsAnInvolution)
{
	const Vector3f ned(1.f, -2.f, -3.f);
	const Vector3f paper = nedToPaper(ned);
	EXPECT_FLOAT_EQ(paper(0), 1.f);
	EXPECT_FLOAT_EQ(paper(1), -2.f);
	EXPECT_FLOAT_EQ(paper(2), 3.f); // altitude 3 m is z = +3 in the paper
	const Vector3f back = paperToNed(paper);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(back(i), ned(i));
	}
}

TEST(TrajControlMath, EulerRatesRoundTrip)
{
	for (float t = 0.f; t < 10.f; t += 0.37f) {
		const Vector3f euler = eulerAt(t);
		const Vector3f rate = eulerRateAt(t);
		const Vector3f back = bodyRatesToEulerRates(euler, eulerRatesToBodyRates(euler, rate));

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(back(i), rate(i), 1e-5f) << "t=" << t;
		}
	}
}

TEST(TrajControlMath, EulerRatesAtLevelAttitudeAreBodyRates)
{
	const Vector3f euler(0.f, 0.f, 1.2f);
	const Vector3f omega = eulerRatesToBodyRates(euler, Vector3f(0.1f, 0.2f, 0.3f));
	EXPECT_NEAR(omega(0), 0.1f, 1e-6f);
	EXPECT_NEAR(omega(1), 0.2f, 1e-6f);
	EXPECT_NEAR(omega(2), 0.3f, 1e-6f);
}

TEST(TrajControlMath, BodyAccelMatchesDerivativeOfBodyRate)
{
	const float h = 1e-3f;

	for (float t = 0.5f; t < 9.f; t += 0.41f) {
		const Vector3f w_minus = eulerRatesToBodyRates(eulerAt(t - h), eulerRateAt(t - h));
		const Vector3f w_plus = eulerRatesToBodyRates(eulerAt(t + h), eulerRateAt(t + h));
		const Vector3f numeric = (w_plus - w_minus) / (2.f * h);
		const Vector3f analytic = eulerAccelToBodyAccel(eulerAt(t), eulerRateAt(t), eulerAccelAt(t));

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(analytic(i), numeric(i), 5e-3f) << "t=" << t << " axis=" << i;
		}
	}
}

TEST(TrajControlMath, HoverCommandGivesHoverThrustAndZeroTorque)
{
	VehicleModel model;
	Limits limits;
	Command cmd;
	cmd.thrust = model.mass * kGravity;

	const Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_NEAR(sp.thrust_norm, model.hover_thrust, 1e-6f);
	EXPECT_NEAR(sp.thrust(2), -model.hover_thrust, 1e-6f);
	EXPECT_FLOAT_EQ(sp.thrust(0), 0.f);
	EXPECT_FLOAT_EQ(sp.thrust(1), 0.f);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(sp.torque(i), 0.f);
	}

	EXPECT_FALSE(sp.thrust_saturated);
	EXPECT_FALSE(sp.torque_saturated);
}

TEST(TrajControlMath, TorqueScalesWithInertiaAndAcceleration)
{
	VehicleModel model;
	Limits limits;
	Command cmd;
	cmd.thrust = model.mass * kGravity;
	cmd.euler_accel = Vector3f(5.f, -4.f, 0.5f);

	// Level, not rotating: euler accelerations are body accelerations
	const Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_NEAR(sp.torque(0), model.inertia(0) * 5.f / model.torque_max(0), 1e-6f);
	EXPECT_NEAR(sp.torque(1), model.inertia(1) * -4.f / model.torque_max(1), 1e-6f);
	EXPECT_NEAR(sp.torque(2), model.inertia(2) * 0.5f / model.torque_max(2), 1e-6f);
	EXPECT_FALSE(sp.torque_saturated);
}

TEST(TrajControlMath, ThrustIsSaturated)
{
	VehicleModel model;
	Limits limits;
	Command cmd;

	cmd.thrust = 1000.f;
	Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_FLOAT_EQ(sp.thrust_norm, limits.thrust_max);
	EXPECT_TRUE(sp.thrust_saturated);

	cmd.thrust = 0.f;
	sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_FLOAT_EQ(sp.thrust_norm, limits.thrust_min);
	EXPECT_TRUE(sp.thrust_saturated);
}

TEST(TrajControlMath, AngularAccelerationIsLimited)
{
	VehicleModel model;
	Limits limits;
	Command cmd;
	cmd.thrust = model.mass * kGravity;
	cmd.euler_accel = Vector3f(1000.f, 0.f, -1000.f);

	const Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_TRUE(sp.torque_saturated);
	EXPECT_NEAR(sp.torque(0), model.inertia(0) * limits.alpha_max(0) / model.torque_max(0), 1e-6f);
	EXPECT_NEAR(sp.torque(2), -model.inertia(2) * limits.alpha_max(2) / model.torque_max(2), 1e-6f);
}

TEST(TrajControlMath, NormalizedTorqueNeverExceedsOne)
{
	VehicleModel model;
	model.torque_max = Vector3f(0.01f, 0.01f, 0.01f); // tiny authority
	Limits limits;
	Command cmd;
	cmd.thrust = model.mass * kGravity;
	cmd.euler_accel = Vector3f(10.f, -10.f, 1.f);

	const Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_TRUE(sp.torque_saturated);

	for (int i = 0; i < 3; i++) {
		EXPECT_LE(fabsf(sp.torque(i)), 1.f);
	}
}

TEST(TrajControlMath, NonFiniteCommandFailsSafe)
{
	VehicleModel model;
	Limits limits;
	Command cmd;
	cmd.thrust = NAN;
	cmd.euler_accel = Vector3f(NAN, 1.f, 2.f);

	const Setpoints sp = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_FLOAT_EQ(sp.thrust_norm, limits.thrust_min);
	EXPECT_TRUE(sp.thrust_saturated);
	EXPECT_TRUE(sp.torque_saturated);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(sp.torque(i), 0.f);
	}

	// non finite torque with a finite thrust: torque zeroed
	cmd.thrust = model.mass * kGravity;
	const Setpoints sp2 = toSetpoints(model, limits, Vector3f(), Vector3f(), cmd);
	EXPECT_TRUE(sp2.torque_saturated);
	EXPECT_TRUE(PX4_ISFINITE(sp2.torque(0)));
}

TEST(TrajControlMath, TiltAngle)
{
	EXPECT_NEAR(tiltAngle(matrix::Quatf()), 0.f, 1e-6f);
	EXPECT_NEAR(tiltAngle(matrix::Quatf(matrix::Eulerf(0.4f, 0.f, 1.f))), 0.4f, 1e-5f);
	EXPECT_NEAR(tiltAngle(matrix::Quatf(matrix::Eulerf(0.f, -0.3f, 2.f))), 0.3f, 1e-5f);
}

TEST(TrajControlMath, LimitAbs)
{
	float v = 5.f;
	EXPECT_TRUE(limitAbs(v, 2.f));
	EXPECT_FLOAT_EQ(v, 2.f);
	v = -5.f;
	EXPECT_TRUE(limitAbs(v, 2.f));
	EXPECT_FLOAT_EQ(v, -2.f);
	v = 1.f;
	EXPECT_FALSE(limitAbs(v, 2.f));
	EXPECT_FLOAT_EQ(v, 1.f);
	v = INFINITY;
	EXPECT_TRUE(limitAbs(v, 2.f));
	EXPECT_FLOAT_EQ(v, 0.f);
}

TEST(TrajControlMath, IsFresh)
{
	EXPECT_TRUE(isFresh(1000000, 900000, 200000));
	EXPECT_TRUE(isFresh(1000000, 800000, 200000));
	EXPECT_FALSE(isFresh(1000000, 700000, 200000));
	EXPECT_FALSE(isFresh(1000000, 0, 200000));       // never published
	EXPECT_FALSE(isFresh(1000000, 1000001, 200000)); // timestamp in the future
}

TEST(TrajControlMath, ExtrapolateMatchesPolynomial)
{
	// p(t) = 1 + 2t + 3t^2 + 4t^3  ->  v = 2 + 6t + 12t^2, a = 6 + 24t, j = 24 at t = 0.5
	const float t = 0.5f;
	const float dt = 0.02f;
	const auto P = [](float x) { return 1.f + 2.f * x + 3.f * x * x + 4.f * x * x * x; };
	const Vector3f p(P(t), 2.f * P(t), -P(t));
	const Vector3f v(2.f + 6.f * t + 12.f * t * t, 0.f, 0.f);
	const Vector3f a(6.f + 24.f * t, 0.f, 0.f);
	const Vector3f j(24.f, 0.f, 0.f);
	const Vector3f next = extrapolate(p, v, a, j, dt);
	EXPECT_NEAR(next(0), P(t + dt), 1e-5f); // cubic polynomial: exact
}
