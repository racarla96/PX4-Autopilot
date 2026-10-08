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
 * Unit tests of the FL law.
 * Run only these using: make tests TESTFILTER=FlControlLaw
 */

#include <gtest/gtest.h>
#include <math.h>

#include "FlControlLaw.hpp"

using namespace fl_control;

namespace
{

constexpr double kTs = 0.02;
constexpr double kG = 9.81;

// Reference of the study (build_project_config.m, Ts = 0.02 s): circle + climb, then hold and descend. Sample i.
struct PaperRef {
	double x, y, z, psi, dx, dy, dz, dpsi;
};

PaperRef paperRef(int i)
{
	const double R = 2., vz1 = 0.05, om = 0.3;
	const int n1 = 2001; // samples of leg 1 (t = 0 .. 40 s)
	PaperRef r;

	if (i < n1) {
		const double t = i * kTs;
		r.x = 1. + R * cos(t * om + M_PI / 2.);
		r.dx = -om * R * sin(t * om + M_PI / 2.);
		r.y = 5. + R * sin(t * om - M_PI / 2.);
		r.dy = om * R * cos(t * om - M_PI / 2.);
		r.z = vz1 * t;
		r.dz = vz1;
		r.psi = om * t;
		r.dpsi = om;

	} else {
		const double t1 = 40.;
		const double t = (i - n1) * kTs; // leg 2 restarts its own clock
		const double z_end = vz1 * t1;
		r.x = 1. + R * cos(t1 * om + M_PI / 2.);
		r.y = 5. + R * sin(t1 * om - M_PI / 2.);
		r.dx = 0.;
		r.dy = 0.;
		r.psi = om * t1;
		r.dpsi = 0.;
		const double vz2 = -z_end / 40.;
		r.z = z_end + vz2 * t;
		r.dz = vz2;
	}

	return r;
}

double sat(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Plant of the study's FL Simulink model (ode1, step Ts): state [x xd y yd z zd phi phid theta thetad psi psid]
void eulerPlant(double X[12], double u, double tau_phi, double tau_theta, double tau_psi, double M)
{
	const double sp = sin(X[6]), cp = cos(X[6]), st = sin(X[8]), ct = cos(X[8]);
	const double f[12] = {X[1], -u * st / M, X[3], u * ct * sp / M, X[5], u * ct * cp / M - kG,
			      X[7], tau_phi, X[9], tau_theta, X[11], tau_psi
			     };

	for (int i = 0; i < 12; i++) {
		X[i] += kTs * f[i];
	}
}

State toState(const double X[12])
{
	State s;
	s.position[0] = X[0]; s.position[1] = X[2]; s.position[2] = X[4];
	s.velocity[0] = X[1]; s.velocity[1] = X[3]; s.velocity[2] = X[5];
	s.euler[0] = X[6]; s.euler[1] = X[8]; s.euler[2] = X[10];
	s.euler_rate[0] = X[7]; s.euler_rate[1] = X[9]; s.euler_rate[2] = X[11];
	return s;
}

} // namespace

TEST(FlControlLaw, ClosedLoopMatchesSimulinkModel)
{
	// x, y, z, psi of modelo_dinamico_FL.slx (MATLAB R2026b, run_simulation.m, Ts = 0.02 s) at sample k
	struct Golden { int k; float x, y, z, psi; };
	const Golden golden[] = {
		{0, 2.0000000f, 0.0000000f, 0.0000000f, 0.0000000f},
		{100, 0.8402872f, 1.3229935f, 0.0712730f, 0.4599523f},
		{250, -1.3830774f, 5.1161034f, 0.2738510f, 1.4552079f},
		{500, 0.5374202f, 7.3288772f, 0.5046727f, 2.9960823f},
		{1000, 1.7398064f, 2.7979409f, 0.9998946f, 5.9999741f},
		{1500, -0.0347180f, 7.0702820f, 1.4999977f, 8.9999998f},
		{2000, 2.3098902f, 3.0989502f, 2.0000003f, 12.0000000f},
		{2100, 1.6953530f, 2.9248568f, 1.9809919f, 12.1442669f},
		{2250, 2.1889650f, 3.4537815f, 1.7338489f, 12.0461483f},
		{3000, 2.0726467f, 3.3110817f, 1.0011657f, 12.0000267f},
		{3999, 2.0731462f, 3.3122862f, 0.0019993f, 12.0000000f},
	};

	const float M = 1.f;
	const float Ts = static_cast<float>(kTs);
	const Gains gains; // first trajectory
	FlControlLaw law;
	law.reset(M * static_cast<float>(kG));

	double X[12] = {2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
	size_t next = 0;

	for (int k = 0; k < 4000; k++) {
		if (next < sizeof(golden) / sizeof(golden[0]) && golden[next].k == k) {
			const Golden &g = golden[next++];
			EXPECT_NEAR(X[0], g.x, 5e-3) << "k=" << k;
			EXPECT_NEAR(X[2], g.y, 5e-3) << "k=" << k;
			EXPECT_NEAR(X[4], g.z, 5e-3) << "k=" << k;
			EXPECT_NEAR(X[10], g.psi, 5e-3) << "k=" << k;
		}

		const PaperRef r = paperRef(k);
		Reference ref;
		ref.position[0] = r.x; ref.position[1] = r.y; ref.position[2] = r.z;
		ref.velocity[0] = r.dx; ref.velocity[1] = r.dy; ref.velocity[2] = r.dz;
		ref.yaw = r.psi;
		ref.yaw_rate = r.dpsi;

		const Output out = law.step(gains, M, Ts, 0.01f, toState(X), ref);
		ASSERT_FALSE(out.singular) << "k=" << k;
		// limits of the study: u in [0, 20] N, tau_phi and tau_theta +-10, tau_psi +-1 rad/s^2
		eulerPlant(X, sat(out.thrust, 0., 20.), sat(out.euler_accel[0], -10., 10.), sat(out.euler_accel[1], -10., 10.),
			   sat(out.euler_accel[2], -1., 1.), M);
	}

	EXPECT_EQ(next, sizeof(golden) / sizeof(golden[0]));
}

TEST(FlControlLaw, DecouplingAndDriftMatchExactDerivative)
{
	// Exact fourth derivative of x, y, z for smooth u(t), phi(t), theta(t): a''(t) must equal G * [u_ddot, phi_ddot,
	// theta_ddot, 0] + f, where a is the translational acceleration of the plant (m = M).
	const double M = 1.7;
	const auto u = [](double t) { return 12. + 2. * sin(0.8 * t); };
	const auto phi = [](double t) { return 0.2 * sin(0.9 * t); };
	const auto theta = [](double t) { return 0.15 * cos(0.7 * t); };
	const auto acc = [&](double t, int axis) {
		const double uu = u(t), sp = sin(phi(t)), cp = cos(phi(t)), st = sin(theta(t)), ct = cos(theta(t));
		return axis == 0 ? -uu * st / M : (axis == 1 ? uu * ct * sp / M : uu * ct * cp / M - kG);
	};

	for (double t = 0.3; t < 8.; t += 0.61) {
		const double h = 1e-3;
		const float euler[3] = {static_cast<float>(phi(t)), static_cast<float>(theta(t)), 0.f};
		const float rate[3] = {static_cast<float>(0.18 * cos(0.9 * t)), static_cast<float>(-0.105 * sin(0.7 * t)), 0.f};
		const float u_dot = static_cast<float>(1.6 * cos(0.8 * t));
		const double inputs[4] = {-1.28 * sin(0.8 * t), -0.162 * sin(0.9 * t), -0.0735 * cos(0.7 * t), 0.};

		float G[4][4];
		float f[3];
		decouplingMatrix(static_cast<float>(M), static_cast<float>(u(t)), euler, G);
		driftTerms(static_cast<float>(M), static_cast<float>(u(t)), u_dot, euler, rate, true, f);

		for (int axis = 0; axis < 3; axis++) {
			const double numeric = (acc(t + h, axis) - 2. * acc(t, axis) + acc(t - h, axis)) / (h * h);
			double model = static_cast<double>(f[axis]);

			for (int j = 0; j < 4; j++) {
				model += static_cast<double>(G[axis][j]) * inputs[j];
			}

			EXPECT_NEAR(model, numeric, 2e-3) << "t=" << t << " axis=" << axis;
		}
	}
}

TEST(FlControlLaw, PaperFzDiffersFromExactOnlyInUdotTerm)
{
	const float euler[3] = {0.2f, -0.1f, 0.3f};
	const float rate[3] = {0.3f, -0.2f, 0.1f};
	float exact[3], paper[3];

	driftTerms(1.f, 10.f, 0.f, euler, rate, true, exact);
	driftTerms(1.f, 10.f, 0.f, euler, rate, false, paper);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(exact[i], paper[i]); // no difference when u_dot = 0
	}

	driftTerms(1.f, 10.f, 0.7f, euler, rate, true, exact);
	driftTerms(1.f, 10.f, 0.7f, euler, rate, false, paper);
	EXPECT_FLOAT_EQ(exact[0], paper[0]);
	EXPECT_FLOAT_EQ(exact[1], paper[1]);
	EXPECT_GT(fabsf(exact[2] - paper[2]), 1e-3f);
}

TEST(FlControlLaw, FilteredDerivative)
{
	const float Ts = 0.02f;
	FilteredDerivative fd;

	// first call: no derivative spike whatever the initial value
	EXPECT_FLOAT_EQ(fd.step(123.f, Ts, 0.01f), 0.f);

	// constant input -> zero derivative
	for (int i = 0; i < 50; i++) {
		EXPECT_NEAR(fd.step(123.f, Ts, 0.01f), 0.f, 1e-5f);
	}

	// ramp of slope 2 -> derivative converges to 2 (the effective time constant is max(0.01, 10 Ts) = 0.2 s)
	float y = 0.f;

	for (int i = 1; i <= 500; i++) {
		y = fd.step(123.f + 2.f * Ts * i, Ts, 0.01f);
	}

	EXPECT_NEAR(y, 2.f, 1e-3f);

	// the effective time constant makes the response to a step slower than T alone would: after one T the output
	// is still close to its initial peak 1 / Te
	FilteredDerivative fd2;
	fd2.step(0.f, Ts, 0.01f);
	EXPECT_NEAR(fd2.step(1.f, Ts, 0.01f), 1.f / 0.2f, 1e-4f);
}

TEST(FlControlLaw, HoverOnTheReferenceKeepsHoverThrust)
{
	FlControlLaw law;
	const float M = 1.5f;
	law.reset(M * 9.81f);
	State s;
	s.position[0] = 1.f; s.position[1] = 2.f; s.position[2] = 3.f;
	Reference r;
	r.position[0] = 1.f; r.position[1] = 2.f; r.position[2] = 3.f;

	for (int k = 0; k < 100; k++) {
		const Output out = law.step(Gains(), M, 0.02f, 0.01f, s, r);
		EXPECT_FALSE(out.singular);
		EXPECT_NEAR(out.thrust, M * 9.81f, 1e-4f);
		EXPECT_NEAR(out.thrust_ddot, 0.f, 1e-4f);

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(out.euler_accel[i], 0.f, 1e-4f);
		}
	}
}

TEST(FlControlLaw, SingularDecouplingGivesZeroInputs)
{
	// u = 0: the thrust columns of G vanish and G cannot be inverted
	FlControlLaw law;
	law.reset(0.f);
	State s;
	Reference r;
	r.position[0] = 1.f;
	const Output out = law.step(Gains(), 1.f, 0.02f, 0.01f, s, r);
	EXPECT_TRUE(out.singular);
	EXPECT_FLOAT_EQ(out.thrust_ddot, 0.f);

	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(out.euler_accel[i], 0.f);
	}
}

TEST(FlControlLaw, ThrustStateIsLimited)
{
	FlControlLaw law;
	law.reset(30.f);
	law.limitThrust(2.f, 20.f);
	EXPECT_FLOAT_EQ(law.thrust(), 20.f);
	law.reset(-1.f);
	law.limitThrust(2.f, 20.f);
	EXPECT_FLOAT_EQ(law.thrust(), 2.f);
}
