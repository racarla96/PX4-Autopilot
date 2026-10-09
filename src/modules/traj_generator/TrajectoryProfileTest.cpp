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
 * Unit tests for the LAB/FL reference trajectory generator.
 * Run only these using: make tests TESTFILTER=TrajectoryProfile
 */

#include <gtest/gtest.h>
#include <math.h>

#include "TrajectoryProfile.hpp"

using namespace traj_generator;

namespace
{

ProfileStart makeStart()
{
	ProfileStart s;
	s.x = 3.f;
	s.y = -2.f;
	s.z = -1.5f;
	s.yaw = 0.7f;
	return s;
}

ProfileParams makeParams(TrajectoryType type)
{
	ProfileParams p;
	p.type = type;
	p.amplitude = 1.5f;
	p.frequency = 0.2f;
	p.altitude = 3.f;
	p.altitude_time = 4.f;
	p.duration = 30.f;
	p.ramp = 4.f;
	return p;
}

} // namespace

class TrajectoryProfileTest : public ::testing::TestWithParam<TrajectoryType> {};

TEST_P(TrajectoryProfileTest, StartsAtStartPointAtRest)
{
	const ProfileStart start = makeStart();
	const ProfileReference ref = evaluate(makeParams(GetParam()), start, 0.f);

	EXPECT_NEAR(ref.position[0], start.x, 1e-5f);
	EXPECT_NEAR(ref.position[1], start.y, 1e-5f);
	EXPECT_NEAR(ref.position[2], start.z, 1e-5f);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(ref.velocity[i], 0.f, 1e-5f);
		EXPECT_NEAR(ref.acceleration[i], 0.f, 1e-5f);
	}

	EXPECT_NEAR(ref.yaw, start.yaw, 1e-6f);
	EXPECT_FALSE(ref.finished);
}

TEST_P(TrajectoryProfileTest, EndsAtRestAndFinishes)
{
	const ProfileParams params = makeParams(GetParam());
	const ProfileReference at_end = evaluate(params, makeStart(), params.duration);
	const ProfileReference after = evaluate(params, makeStart(), params.duration + 10.f);

	EXPECT_TRUE(at_end.finished);
	EXPECT_TRUE(after.finished);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(at_end.velocity[i], 0.f, 1e-4f);
		EXPECT_NEAR(after.velocity[i], 0.f, 1e-6f);
		EXPECT_NEAR(after.acceleration[i], 0.f, 1e-6f);
		EXPECT_NEAR(after.position[i], at_end.position[i], 1e-4f);
	}

	EXPECT_NEAR(after.position[2], -params.altitude, 1e-4f);
}

TEST_P(TrajectoryProfileTest, DerivativesAreConsistent)
{
	// velocity ~ d(position)/dt, acceleration ~ d(velocity)/dt, jerk ~ d(acceleration)/dt
	const ProfileParams params = makeParams(GetParam());
	const ProfileStart start = makeStart();
	const float h = 1e-2f;

	for (float t = 0.5f; t < params.duration - 0.5f; t += 0.37f) {
		const ProfileReference a = evaluate(params, start, t - h);
		const ProfileReference b = evaluate(params, start, t + h);
		const ProfileReference c = evaluate(params, start, t);

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(c.velocity[i], (b.position[i] - a.position[i]) / (2.f * h), 2e-3f) << "t=" << t << " axis=" << i;
			EXPECT_NEAR(c.acceleration[i], (b.velocity[i] - a.velocity[i]) / (2.f * h), 2e-3f) << "t=" << t << " axis=" << i;
			EXPECT_NEAR(c.jerk[i], (b.acceleration[i] - a.acceleration[i]) / (2.f * h), 5e-3f) << "t=" << t << " axis=" << i;
		}
	}
}

INSTANTIATE_TEST_SUITE_P(AllTypes, TrajectoryProfileTest,
			 ::testing::Values(TrajectoryType::Hover, TrajectoryType::Circle, TrajectoryType::Figure8));

TEST(TrajectoryProfile, CircleStaysOnCircleAtCruise)
{
	ProfileParams params = makeParams(TrajectoryType::Circle);
	params.altitude = 0.f;
	const ProfileStart start = makeStart();

	// Centre is amplitude north of the start point
	const float cx = start.x - params.amplitude;
	const float cy = start.y;

	for (float t = 0.f; t < params.duration; t += 0.9f) {
		const ProfileReference ref = evaluate(params, start, t);
		const float r = hypotf(ref.position[0] - start.x + params.amplitude, ref.position[1] - cy);
		EXPECT_NEAR(r, params.amplitude, 1e-4f);
		(void)cx;
	}
}

TEST(TrajectoryProfile, AltitudeHeldWhenNotRequested)
{
	ProfileParams params = makeParams(TrajectoryType::Circle);
	params.altitude = 0.f;
	const ProfileStart start = makeStart();

	for (float t = 0.f; t < 40.f; t += 1.3f) {
		const ProfileReference ref = evaluate(params, start, t);
		EXPECT_FLOAT_EQ(ref.position[2], start.z);
		EXPECT_FLOAT_EQ(ref.velocity[2], 0.f);
	}
}

TEST(TrajectoryProfile, InfiniteDurationNeverFinishes)
{
	ProfileParams params = makeParams(TrajectoryType::Figure8);
	params.duration = 0.f;
	EXPECT_FALSE(evaluate(params, makeStart(), 1.0e5f).finished);
}

TEST(TrajectoryProfile, DeterministicForSameInputs)
{
	const ProfileParams params = makeParams(TrajectoryType::Figure8);
	const ProfileReference a = evaluate(params, makeStart(), 12.34f);
	const ProfileReference b = evaluate(params, makeStart(), 12.34f);

	for (int i = 0; i < 3; i++) {
		EXPECT_EQ(a.position[i], b.position[i]);
		EXPECT_EQ(a.jerk[i], b.jerk[i]);
	}
}

TEST(TrajectoryProfile, PaperMatchesMatlabReference)
{
	// Samples of x_ref - x_ref(0), y_ref - y_ref(0), z_ref and psi_ref from run_simulation.m (Ts = 0.02 s).
	// The paper's z points up, the generator publishes NED. The MATLAB arrays repeat the sample at t = 40 s,
	// so leg-2 samples are one sample (0.02 s) late: z is compared with 2 mm tolerance.
	struct Sample { float t, x, y, z_up, psi; };
	const Sample samples[] = {
		{0.00f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f},
		{2.00f, -1.1292849f, 0.3493288f, 0.1000000f, 0.6000000f},
		{10.00f, -0.2822400f, 3.9799850f, 0.5000000f, 3.0000000f},
		{25.00f, -1.8760000f, 1.3067294f, 1.2500000f, 7.5000000f},
		{39.98f, 1.0832527f, 0.3187613f, 1.9990000f, 11.9940000f},
		{40.00f, 1.0731458f, 0.3122921f, 2.0000000f, 12.0000000f},
		{50.00f, 1.0731458f, 0.3122921f, 1.5010000f, 12.0000000f},
		{60.00f, 1.0731458f, 0.3122921f, 1.0010000f, 12.0000000f},
		{79.98f, 1.0731458f, 0.3122921f, 0.0020000f, 12.0000000f},
	};

	ProfileParams params;
	params.type = TrajectoryType::Paper;
	const ProfileStart start = makeStart();

	for (const Sample &s : samples) {
		const ProfileReference ref = evaluate(params, start, s.t);
		EXPECT_NEAR(ref.position[0] - start.x, s.x, 1e-4f) << "t=" << s.t;
		EXPECT_NEAR(ref.position[1] - start.y, s.y, 1e-4f) << "t=" << s.t;
		EXPECT_NEAR(start.z - ref.position[2], s.z_up, 2e-3f) << "t=" << s.t;
		EXPECT_NEAR(ref.yaw - start.yaw, s.psi, 1e-4f) << "t=" << s.t;
	}
}

TEST(TrajectoryProfile, PaperDerivativesAreConsistentWithinLegs)
{
	ProfileParams params;
	params.type = TrajectoryType::Paper;
	const ProfileStart start = makeStart();
	const float h = 1e-2f;

	// Away from the velocity step at t = 40 s
	for (float t = 0.5f; t < 79.f; t += 0.73f) {
		if (fabsf(t - 40.f) < 0.1f) {
			continue;
		}

		const ProfileReference a = evaluate(params, start, t - h);
		const ProfileReference b = evaluate(params, start, t + h);
		const ProfileReference c = evaluate(params, start, t);

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(c.velocity[i], (b.position[i] - a.position[i]) / (2.f * h), 3e-3f) << "t=" << t << " axis=" << i;
			EXPECT_NEAR(c.acceleration[i], (b.velocity[i] - a.velocity[i]) / (2.f * h), 3e-3f) << "t=" << t << " axis=" << i;
			EXPECT_NEAR(c.jerk[i], (b.acceleration[i] - a.acceleration[i]) / (2.f * h), 3e-3f) << "t=" << t << " axis=" << i;
		}
	}
}

TEST(TrajectoryProfile, PaperFinishesAtRestAtStartAltitude)
{
	ProfileParams params;
	params.type = TrajectoryType::Paper;
	const ProfileStart start = makeStart();
	const ProfileReference end = evaluate(params, start, 80.f);
	EXPECT_TRUE(end.finished);
	EXPECT_FALSE(evaluate(params, start, 79.9f).finished);
	EXPECT_NEAR(end.position[2], start.z, 1e-4f);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(end.velocity[i], 0.f);
	}
}

TEST(TrajectoryProfile, Paper2MatchesTheExternalReferenceFile)
{
	// Samples of data/referencia_externa.mat (paper frame: z up): t, x, y, z, psi, vx, vy, vz, psi_dot
	struct Sample { float t, x, y, z_up, psi, vx, vy, vz_up, dpsi; };
	const Sample samples[] = {
		{0.00f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f},
		{1.00f, 0.0000000f, 0.0000000f, 0.1667200f, 0.1047533f, 0.0000000f, 0.0000000f, 0.5734400f, 0.3603030f},
		{2.50f, 0.0000000f, 0.0000000f, 2.5000000f, 1.5707963f, 0.0000000f, 0.0000000f, 2.1875000f, 1.3744468f},
		{4.00f, 0.0000000f, 0.0000000f, 4.8332800f, 3.0368394f, 0.0000000f, 0.0000000f, 0.5734400f, 0.3603030f},
		{5.00f, 0.0000000f, 0.0000000f, 5.0000000f, 3.1415927f, 0.0000000f, 0.0000000f, 0.0000000f, 0.0000000f},
		{7.50f, 2.5000000f, 0.0000000f, 5.0000000f, 1.5707963f, 2.1875000f, 0.0000000f, 0.0000000f, -1.3744468f},
		{12.00f, 5.0000000f, 2.8979200f, 5.0000000f, 0.9104084f, 0.0000000f, 3.8707200f, 0.0000000f, 1.2160226f},
		{16.00f, 4.8332800f, 10.0000000f, 5.0000000f, 3.0368394f, -0.5734400f, 0.0000000f, 0.0000000f, -0.3603030f},
		{22.50f, 0.0000000f, 10.0000000f, 4.0000000f, 1.5707963f, 0.0000000f, 0.0000000f, -0.8750000f, 1.3744468f},
		{27.50f, 2.5000000f, 10.0000000f, 3.0000000f, 1.5707963f, 2.1875000f, 0.0000000f, 0.0000000f, -1.3744468f},
		{31.00f, 5.0000000f, 10.0000000f, 2.9006532f, 0.1040357f, 0.0000000f, 0.0000000f, -0.3418364f, 0.3579703f},
		{34.00f, 5.0000000f, 10.0000000f, 0.1028037f, 3.0339368f, 0.0000000f, 0.0000000f, -0.3495661f, 0.3660648f},
		{35.00f, 5.0000000f, 10.0000000f, 0.0000000f, 3.1415927f, 0.0000000f, 0.0000000f, -0.0000007f, 0.0000007f},
	};

	ProfileParams params;
	params.type = TrajectoryType::Paper2;
	const ProfileStart start = makeStart();

	for (const Sample &s : samples) {
		const ProfileReference ref = evaluate(params, start, s.t);
		const float tol = (s.t > 30.f) ? 1e-2f : 1e-4f; // the last move of the file is 8 mm off the exact polynomial
		EXPECT_NEAR(ref.position[0] - start.x, s.x, tol) << "t=" << s.t;
		EXPECT_NEAR(ref.position[1] - start.y, s.y, tol) << "t=" << s.t;
		EXPECT_NEAR(start.z - ref.position[2], s.z_up, tol) << "t=" << s.t;
		EXPECT_NEAR(ref.yaw - start.yaw, s.psi, tol) << "t=" << s.t;
		EXPECT_NEAR(ref.velocity[0], s.vx, tol) << "t=" << s.t;
		EXPECT_NEAR(ref.velocity[1], s.vy, tol) << "t=" << s.t;
		EXPECT_NEAR(-ref.velocity[2], s.vz_up, tol) << "t=" << s.t;
		EXPECT_NEAR(ref.yaw_rate, s.dpsi, tol) << "t=" << s.t;
	}

	EXPECT_TRUE(evaluate(params, start, 35.f).finished);
	EXPECT_FALSE(evaluate(params, start, 34.9f).finished);
}

TEST(TrajectoryProfile, Paper2DerivativesAreConsistent)
{
	ProfileParams params;
	params.type = TrajectoryType::Paper2;
	const ProfileStart start = makeStart();
	const float h = 3e-2f; // single precision: the 7th order polynomial limits the finite difference accuracy

	for (float t = 0.3f; t < 34.7f; t += 0.37f) {
		const ProfileReference a = evaluate(params, start, t - h);
		const ProfileReference b = evaluate(params, start, t + h);
		const ProfileReference c = evaluate(params, start, t);

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR((b.position[i] - a.position[i]) / (2.f * h), c.velocity[i], 5e-3f) << "t=" << t;
			EXPECT_NEAR((b.velocity[i] - a.velocity[i]) / (2.f * h), c.acceleration[i], 5e-3f) << "t=" << t;
			EXPECT_NEAR((b.acceleration[i] - a.acceleration[i]) / (2.f * h), c.jerk[i], 2e-2f) << "t=" << t;
		}

		EXPECT_NEAR((b.yaw - a.yaw) / (2.f * h), c.yaw_rate, 2e-3f) << "t=" << t;
	}
}
