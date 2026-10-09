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
 * Unit tests for the safety monitor of the LAB and FL modes.
 * Run only these using: make tests TESTFILTER=SafetyMonitor
 */

#include <gtest/gtest.h>

#include "SafetyMonitor.hpp"

using namespace traj_control;

namespace
{

SafetySample good()
{
	SafetySample s;
	s.position[2] = 2.5f;
	return s;
}

} // namespace

TEST(SafetyMonitor, NominalFlightNeverTrips)
{
	SafetyMonitor m;
	const SafetyLimits limits;

	for (int i = 0; i < 1000; i++) {
		SafetySample s = good();
		s.error_norm = 0.6f;
		s.tilt = 0.3f;
		EXPECT_EQ(m.update(0.02f * static_cast<float>(i), limits, s), Trip::None);
	}
}

TEST(SafetyMonitor, InputsLostTripsOnlyAfterTheTimeout)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	EXPECT_EQ(m.update(0.f, limits, s), Trip::None);

	s.inputs_ok = false;
	EXPECT_EQ(m.update(1.0f, limits, s), Trip::None);
	EXPECT_EQ(m.update(1.3f, limits, s), Trip::None);
	EXPECT_EQ(m.update(1.6f, limits, s), Trip::InputsLost);
}

TEST(SafetyMonitor, ShortInputDropoutIsForgiven)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	m.update(0.f, limits, s);
	s.inputs_ok = false;
	m.update(1.0f, limits, s);
	s.inputs_ok = true;
	m.update(1.2f, limits, s);
	s.inputs_ok = false;
	EXPECT_EQ(m.update(1.5f, limits, s), Trip::None); // the clock restarted at 1.5 s
	EXPECT_EQ(m.update(1.9f, limits, s), Trip::None);
	EXPECT_EQ(m.update(2.1f, limits, s), Trip::InputsLost);
}

TEST(SafetyMonitor, LargeErrorMustPersist)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	s.error_norm = 4.f;
	EXPECT_EQ(m.update(0.f, limits, s), Trip::None);
	s.error_norm = 1.f;
	EXPECT_EQ(m.update(0.2f, limits, s), Trip::None); // a spike
	s.error_norm = 4.f;
	EXPECT_EQ(m.update(0.4f, limits, s), Trip::None);
	EXPECT_EQ(m.update(0.8f, limits, s), Trip::ErrorTooLarge);
}

TEST(SafetyMonitor, GeometricLimitsAreRelativeToTheEntryPoint)
{
	SafetyLimits limits;
	limits.distance_max = 10.f;
	limits.altitude_max = 6.f;

	SafetyMonitor far_monitor;
	SafetySample s = good();
	s.position[0] = 100.f; // the entry point is far from the origin: only the displacement counts
	s.position[1] = -50.f;
	EXPECT_EQ(far_monitor.update(0.f, limits, s), Trip::None);
	s.position[0] = 106.f;
	s.position[1] = -42.f;
	EXPECT_EQ(far_monitor.update(0.1f, limits, s), Trip::None); // 10 m exactly
	s.position[0] = 109.f;
	EXPECT_EQ(far_monitor.update(0.2f, limits, s), Trip::TooFar);

	SafetyMonitor high_monitor;
	s = good();
	EXPECT_EQ(high_monitor.update(0.f, limits, s), Trip::None);
	s.position[2] = 2.5f + 5.9f;
	EXPECT_EQ(high_monitor.update(0.1f, limits, s), Trip::None);
	s.position[2] = 2.5f + 6.1f;
	EXPECT_EQ(high_monitor.update(0.2f, limits, s), Trip::TooHigh);
}

TEST(SafetyMonitor, TiltTripsAtOnce)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	EXPECT_EQ(m.update(0.f, limits, s), Trip::None);
	s.tilt = 1.3f;
	EXPECT_EQ(m.update(0.02f, limits, s), Trip::TiltTooLarge);
}

TEST(SafetyMonitor, ControllerFailureNeedsAFewSamples)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	s.controller_ok = false;
	EXPECT_EQ(m.update(0.f, limits, s), Trip::None);
	EXPECT_EQ(m.update(0.1f, limits, s), Trip::None);
	EXPECT_EQ(m.update(0.25f, limits, s), Trip::ControllerFailure);
}

TEST(SafetyMonitor, StaysTrippedUntilReset)
{
	SafetyMonitor m;
	const SafetyLimits limits;
	SafetySample s = good();
	s.tilt = 1.5f;
	EXPECT_EQ(m.update(0.f, limits, s), Trip::TiltTooLarge);
	EXPECT_EQ(m.update(1.f, limits, good()), Trip::TiltTooLarge);
	m.reset();
	EXPECT_EQ(m.update(2.f, limits, good()), Trip::None);
}

TEST(SafetyMonitor, ZeroDisablesACheck)
{
	SafetyMonitor m;
	SafetyLimits limits;
	limits.error_max = 0.f;
	limits.distance_max = 0.f;
	limits.altitude_max = 0.f;
	limits.tilt_abort = 0.f;
	SafetySample s = good();
	s.error_norm = 100.f;
	s.tilt = 3.f;
	s.position[0] = 1e4f;
	s.position[2] = 1e4f;

	for (int i = 0; i < 100; i++) {
		EXPECT_EQ(m.update(0.1f * static_cast<float>(i), limits, s), Trip::None);
	}
}
