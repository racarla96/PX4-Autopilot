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
 * Unit tests of the LAB law.
 * Run only these using: make tests TESTFILTER=LabControlLaw
 */

#include <gtest/gtest.h>
#include <math.h>

#include "LabControlLaw.hpp"

using namespace lab_control;

namespace
{

constexpr float kTs = 0.02f;
constexpr float kMass = 1.f;
constexpr float kG = 9.81f;
constexpr float kNoTiltLimit = 1.5f;

struct GoldenCase {
	float pos[3];
	float vel[3];
	float euler[3];
	float rate[3];
	Reference now;
	Reference next;
	float out[4]; // u, tau_phi, tau_theta, tau_psi
};

// Outputs of controllers/LAB_controller.m (MATLAB R2026b) for the same inputs, gains of the first trajectory.
const GoldenCase kGolden[] = {
	{{-0.847383421f,0.559837584f,-0.123181537f}, {0.223465178f,0.477989512f,0.0384958704f}, {0.000448185464f,-0.171179547f,-1.4549408f}, {-7.04995047e-05f,0.107537998f,0.182243422f},
	 {{-0.906912855f,0.342805758f,-0.229108737f}, -1.32557899f}, {{-0.894625049f,0.334207319f,-0.230545019f}, -1.32533f},
	 {8.17715107f,-1.12780821f,0.408891661f,-0.262782262f}},
	{{0.201097835f,0.900259001f,-0.539394242f}, {0.0484899192f,0.409128375f,-0.366830554f}, {0.00936503227f,0.100163944f,1.06194151f}, {-0.0193482842f,-0.177090546f,-0.00554046655f},
	 {{0.13729018f,0.888959578f,-0.606449049f}, 1.08372911f}, {{0.147427719f,0.897019003f,-0.612029209f}, 1.0864896f},
	 {10.3998004f,-0.00745504215f,-0.585365737f,0.386146977f}},
	{{-0.0943141349f,-0.294043268f,0.314798926f}, {-0.129648917f,-0.0409070221f,0.219324123f}, {-0.0348032684f,0.162569308f,-2.00778169f}, {0.144671324f,-0.0465755738f,-0.0441278564f},
	 {{-0.0271242006f,-0.282590168f,0.272241915f}, -2.00047801f}, {{-0.0420813942f,-0.294822297f,0.278523747f}, -1.99351641f},
	 {10.6329286f,-0.817862265f,-0.453722313f,0.989649819f}},
	{{0.910936646f,0.365827709f,-0.893742619f}, {-0.191147315f,0.0925946873f,-0.264879593f}, {0.1859884f,0.17801929f,2.1890673f}, {-0.0166056022f,0.204886029f,-0.221333615f},
	 {{0.815303475f,0.347325906f,-0.772819018f}, 2.13639515f}, {{0.814878232f,0.336432189f,-0.777512922f}, 2.13939934f},
	 {10.7827429f,-2.13921067f,-2.37955378f,0.863017481f}},
	{{-0.668997199f,-0.170196455f,-0.103758685f}, {0.274900376f,0.296390701f,0.022390128f}, {-0.0157478815f,0.111285441f,2.43340825f}, {0.104951262f,0.180287429f,0.263466812f},
	 {{-0.898669294f,0.0176394076f,-0.215477149f}, 2.32755252f}, {{-0.899396359f,0.0265422363f,-0.208959882f}, 2.33414001f},
	 {12.4287635f,-0.154582861f,-1.19986114f,0.0324486326f}},
	{{-0.86149586f,-0.285858743f,0.6256591f}, {-0.072295167f,0.0998544375f,0.228161283f}, {0.128491043f,0.104206048f,-3.09671013f}, {-0.0478458998f,-0.0221182692f,-0.266700298f},
	 {{-0.840774794f,-0.231973368f,0.789885698f}, -3.04896261f}, {{-0.827520516f,-0.243128932f,0.781798618f}, -3.04763787f},
	 {4.23024788f,-2.33800304f,-2.88499941f,0.89202763f}},
	{{-0.551842726f,0.149725175f,-0.660952553f}, {0.282230151f,0.356975626f,-0.466325795f}, {0.013057922f,0.118780543f,2.98539066f}, {-0.135444844f,-0.198539362f,0.226020558f},
	 {{-0.347251494f,-0.00150838009f,-0.690187682f}, 3.03488135f}, {{-0.34067453f,0.00885197467f,-0.700139422f}, 3.04295971f},
	 {9.3313375f,0.574397317f,-0.347606582f,0.50660629f}},
	{{0.0994282416f,-0.670566682f,-0.92894239f}, {-0.218466182f,0.307870846f,-0.455233739f}, {-0.196713398f,-0.0553533409f,-2.74184204f}, {-0.210308218f,-0.286085779f,0.0148319021f},
	 {{0.197776193f,-0.707039936f,-1.11165716f}, -2.59407473f}, {{0.192716909f,-0.70432956f,-1.09843732f}, -2.5916587f},
	 {20.5720133f,2.63342614f,1.5786091f,0.449633035f}},
};

State toState(const GoldenCase &c)
{
	State s;

	for (int i = 0; i < 3; i++) {
		s.position[i] = c.pos[i];
		s.velocity[i] = c.vel[i];
		s.euler[i] = c.euler[i];
		s.euler_rate[i] = c.rate[i];
	}

	return s;
}

} // namespace

TEST(LabControlLaw, MatchesMatlabController)
{
	for (const GoldenCase &c : kGolden) {
		const Output out = compute(Gains(), kMass, kTs, kG, toState(c), c.now, c.next, kNoTiltLimit);
		EXPECT_NEAR(out.thrust, c.out[0], 2e-3f);
		EXPECT_NEAR(out.euler_accel[0], c.out[1], 2e-3f);
		EXPECT_NEAR(out.euler_accel[1], c.out[2], 2e-3f);
		EXPECT_NEAR(out.euler_accel[2], c.out[3], 2e-3f);
		EXPECT_FALSE(out.tilt_saturated);
	}
}

TEST(LabControlLaw, HoverAtReferenceNeedsWeightAndNoTorque)
{
	// On the reference, at rest, level: thrust = m g and no angular acceleration
	State s;
	s.position[0] = 1.f; s.position[1] = -2.f; s.position[2] = 3.f;
	Reference r;
	r.position[0] = 1.f; r.position[1] = -2.f; r.position[2] = 3.f;
	const Output out = compute(Gains(), 1.7f, kTs, kG, s, r, r, kNoTiltLimit);
	EXPECT_NEAR(out.thrust, 1.7f * kG, 1e-3f);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(out.euler_accel[i], 0.f, 1e-4f);
	}

	EXPECT_NEAR(out.phi_ref, 0.f, 1e-6f);
	EXPECT_NEAR(out.theta_ref, 0.f, 1e-6f);
}

TEST(LabControlLaw, ThrustScalesWithMass)
{
	const GoldenCase &c = kGolden[0];
	const Output a = compute(Gains(), 1.f, kTs, kG, toState(c), c.now, c.next, kNoTiltLimit);
	const Output b = compute(Gains(), 3.f, kTs, kG, toState(c), c.now, c.next, kNoTiltLimit);
	EXPECT_NEAR(b.thrust, 3.f * a.thrust, 1e-3f);
	EXPECT_NEAR(a.euler_accel[0], b.euler_accel[0], 1e-6f);
}

TEST(LabControlLaw, YawErrorIsWrapped)
{
	// Reference just past +pi while the vehicle is just before it: the short way round is a tiny error,
	// not a 2 pi turn.
	State s;
	s.euler[2] = 3.1f;
	Reference now;
	now.yaw = -3.1f;
	Reference next = now;
	const Output out = compute(Gains(), kMass, kTs, kG, s, now, next, kNoTiltLimit);
	EXPECT_LT(fabsf(out.euler_accel[2]), 5.f);
	// a positive yaw error gives a positive yaw acceleration: 2*pi - 6.2 = +0.083 rad
	EXPECT_GT(out.euler_accel[2], 0.f);
}

TEST(LabControlLaw, TiltReferencesAreLimited)
{
	State s;
	Reference now;
	now.position[0] = 10.f; // large error to the north
	Reference next = now;
	const float limit = 0.3f;
	const Output out = compute(Gains(), kMass, kTs, kG, s, now, next, limit);
	EXPECT_TRUE(out.tilt_saturated);
	EXPECT_LE(fabsf(out.phi_ref), limit + 1e-6f);
	EXPECT_LE(fabsf(out.theta_ref), limit + 1e-6f);
	// north error -> accelerate north -> nose down in the paper's convention (pitch positive nose up): theta_ref < 0
	// (x_ddot = -u sin(theta))
	EXPECT_LT(out.theta_ref, 0.f);
}

TEST(LabControlLaw, NoSingularityWhenLateralCommandIsZero)
{
	State s;
	Reference r;
	r.position[0] = 0.5f; // only x error: delta_x4 is exactly 0
	const Output out = compute(Gains(), kMass, kTs, kG, s, r, r, kNoTiltLimit);
	EXPECT_TRUE(std::isfinite(out.thrust));
	EXPECT_TRUE(std::isfinite(out.euler_accel[0]));
	EXPECT_TRUE(std::isfinite(out.euler_accel[1]));
	EXPECT_NEAR(out.phi_ref, 0.f, 1e-6f);
	EXPECT_LT(out.theta_ref, 0.f);
}

namespace
{

double paperRefAt(int i, int what) // 0 x, 1 y, 2 z, 3 psi, indices as build_project_config.m (Ts = 0.02 s)
{
	const double Ts = 0.02, R = 2., vz1 = 0.05, om = 0.3;
	const int n1 = 2001;
	double x, y, z, psi;

	if (i < n1) {
		const double t = i * Ts;
		x = 1. + R * cos(t * om + M_PI / 2.);
		y = 5. + R * sin(t * om - M_PI / 2.);
		z = vz1 * t;
		psi = om * t;

	} else {
		const double t = (i - n1) * Ts;
		x = 1. + R * cos(40. * om + M_PI / 2.);
		y = 5. + R * sin(40. * om - M_PI / 2.);
		psi = om * 40.;
		z = vz1 * 40. - (vz1 * 40. / 40.) * t;
	}

	return what == 0 ? x : (what == 1 ? y : (what == 2 ? z : psi));
}

// Plant of modelo_dinamico_LAB.slx: continuous model with the control held over Ts (RK4 with substeps)
void integratePlant(double X[12], double u, double tp, double tt, double ty)
{
	const double Ts = 0.02;
	const int sub = 20;
	const double h = Ts / sub;

	const auto f = [&](const double *s, double *d) {
		d[0] = s[1]; d[1] = -u * sin(s[8]);
		d[2] = s[3]; d[3] = u * cos(s[8]) * sin(s[6]);
		d[4] = s[5]; d[5] = u * cos(s[8]) * cos(s[6]) - 9.81;
		d[6] = s[7]; d[7] = tp; d[8] = s[9]; d[9] = tt; d[10] = s[11]; d[11] = ty;
	};

	for (int n = 0; n < sub; n++) {
		double k1[12], k2[12], k3[12], k4[12], tmp[12];
		f(X, k1);

		for (int i = 0; i < 12; i++) { tmp[i] = X[i] + 0.5 * h * k1[i]; }

		f(tmp, k2);

		for (int i = 0; i < 12; i++) { tmp[i] = X[i] + 0.5 * h * k2[i]; }

		f(tmp, k3);

		for (int i = 0; i < 12; i++) { tmp[i] = X[i] + h * k3[i]; }

		f(tmp, k4);

		for (int i = 0; i < 12; i++) { X[i] += h / 6. * (k1[i] + 2. * k2[i] + 2. * k3[i] + k4[i]); }
	}
}

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace

TEST(LabControlLaw, ClosedLoopMatchesSimulinkModel)
{
	// x, y, z, psi of modelo_dinamico_LAB.slx (MATLAB R2026b, run_simulation.m, Ts = 0.02 s) at sample k
	struct Golden { int k; float x, y, z, psi; };
	const Golden golden[] = {
		{0, 2.0000000f, 0.0000000f, 0.0000000f, 0.0000000f},
		{100, 0.4602636f, 1.4900553f, 0.0994771f, 0.5416734f},
		{250, -1.0650275f, 4.8364038f, 0.2489693f, 1.4918412f},
		{500, 0.5902753f, 7.2734117f, 0.4999380f, 2.9997378f},
		{1000, 1.7054334f, 2.8166377f, 1.0000045f, 5.9999997f},
		{1500, 0.0067413f, 7.0487428f, 1.5000043f, 9.0000000f},
		{2000, 2.2611942f, 3.1268826f, 2.0000033f, 12.0000000f},
		{2100, 1.8947756f, 2.9553544f, 1.9041512f, 12.0583266f},
		{2250, 2.0297353f, 3.3472537f, 1.7515809f, 12.0081588f},
		{3000, 2.0731438f, 3.3122922f, 1.0010002f, 12.0000003f},
		{3999, 2.0731458f, 3.3122921f, 0.0020000f, 12.0000000f},
	};

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

		State s;
		s.position[0] = X[0]; s.position[1] = X[2]; s.position[2] = X[4];
		s.velocity[0] = X[1]; s.velocity[1] = X[3]; s.velocity[2] = X[5];
		s.euler[0] = X[6]; s.euler[1] = X[8]; s.euler[2] = X[10];
		s.euler_rate[0] = X[7]; s.euler_rate[1] = X[9]; s.euler_rate[2] = X[11];

		Reference now, nxt;

		for (int a = 0; a < 3; a++) {
			now.position[a] = paperRefAt(k, a);
			nxt.position[a] = paperRefAt(k + 1, a);
		}

		now.yaw = paperRefAt(k, 3);
		nxt.yaw = paperRefAt(k + 1, 3);

		const Output out = compute(Gains(), 1.f, kTs, kG, s, now, nxt, kNoTiltLimit);
		integratePlant(X, clampd(out.thrust, 0., 20.), clampd(out.euler_accel[0], -10., 10.),
			       clampd(out.euler_accel[1], -10., 10.), clampd(out.euler_accel[2], -1., 1.));
	}

	EXPECT_EQ(next, sizeof(golden) / sizeof(golden[0]));
}

TEST(LabControlLaw, HeadingCompensationRotatesRollAndPitch)
{
	// Vehicle heading East (psi = 90 deg), reference 1 m ahead of it along East: it has to pitch forward, not roll
	State s;
	s.euler[2] = static_cast<float>(M_PI / 2.);
	Reference now;
	now.position[1] = 1.f;
	now.yaw = s.euler[2];
	Reference next = now;

	const Output paper = compute(Gains(), kMass, kTs, kG, s, now, next, kNoTiltLimit);
	EXPECT_GT(fabsf(paper.phi_ref), 0.05f); // as in the paper's frame: roll towards East while heading East
	EXPECT_NEAR(paper.theta_ref, 0.f, 1e-4f);

	Gains g;
	g.heading_compensation = true;
	const Output rotated = compute(g, kMass, kTs, kG, s, now, next, kNoTiltLimit);
	EXPECT_NEAR(rotated.phi_ref, 0.f, 1e-4f);
	EXPECT_LT(rotated.theta_ref, -0.05f); // nose down
	EXPECT_NEAR(fabsf(rotated.theta_ref), fabsf(paper.phi_ref), 1e-3f);
}
