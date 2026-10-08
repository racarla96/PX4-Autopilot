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
