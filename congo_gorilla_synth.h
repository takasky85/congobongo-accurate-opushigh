// license:BSD-3-Clause
// Congo Bongo (Sega 834-5168) gorilla growl synthesizer.  No samples.
//
// Behavioral model of the gorilla circuit (schematic sheet 2): MM5837 noise source -> gate ->
// 3-pole filter around a saturating op-amp (a noisy random-telegraph signal) -> summing amp that
// sets the control voltage of a relaxation VCO (op-amp integrator + Schmitt trigger + Q2) ->
// 330 Hz low-pass -> voltage-controlled amplifier (Sega custom chip G501534, "U15"), whose gain
// control comes from two 4538B monostables and R/C envelope networks.
//
// Everything is derived from the schematic component values and was checked against a transient
// simulation of the full circuit.  U15 has no public documentation: it is modelled as a linear VCA
// with a control threshold of 2.70 V (fitted to a recording of the real sound, see README).
// Noise: the LFSR is the MAME mm5837 one (17 bit, taps 14/17) at 103.2 kHz (12 V supply).
//
// Trigger: falling edge of the 8255 PB1 pin (the 4538B B inputs are active-low edge triggered).
#ifndef CONGO_GORILLA_SYNTH_H
#define CONGO_GORILLA_SYNTH_H
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// GORILLA_TUNABLE (used only by the fitting tools) makes the constants writable at run time.
#ifdef GORILLA_TUNABLE
#define GK static inline double
#define GA static inline double
#else
#define GK static constexpr double
#define GA static constexpr double
#endif

namespace congo_perc {

class gorilla
{
public:
	static constexpr int RATE = 48000;        // output sample rate this model is built for
	static constexpr int OVER = 8;            // internal oversampling of the noise/VCO loop

	void reset()
	{
		m_lfsr = 0x1ffff; m_bit_left = 1.0 / FCLK; m_gate = 0.0;
		m_x[0] = 2.5; m_x[1] = -2.5; m_x[2] = 2.5; m_o8 = 5.0; m_aclp = 5.0;
		m_vc = 8.4; m_Vc = m_vc / 2 - 5.7; m_shi = false;
		m_e1 = m_e2 = 0.0; m_t = 0.0; m_ttrig = -1e9;
		m_zi[0] = m_zi[1] = 0.0; m_lpy = 0.0;
		m_gate = (m_lfsr >> 16 & 1) ? 0.0 : 5.0;
	}

	void set_lfsr(uint32_t s) { m_lfsr = (s & 0x1ffff) ? (s & 0x1ffff) : 1; m_gate = ((m_lfsr >> 16) & 1) ? 0.0 : 5.0; }

	// falling edge of PB1: (re)start both monostables
	void trigger() { m_ttrig = m_t; }

	// one output sample at RATE: volts at SOU (polarity not applied)
	double tick()
	{
		const double dt = 1.0 / RATE, hh = dt / OVER;
		const double el = m_t - m_ttrig;
		const bool g_hi = (el >= T1) && (el < T2);
		const bool q_hi = (el >= 0) && (el < T2);
		const double ch = 1.0 - std::exp(-dt / 1.4e-3);
		m_e1 = g_hi ? m_e1 + (E1_HI - m_e1) * ch : m_e1 * std::exp(-dt / TAU1);
		m_e2 = q_hi ? m_e2 + (E2_HI - m_e2) * ch : m_e2 * std::exp(-dt / TAU2);
		const double vdc = 8.4 - 1.1 * m_e1;
		const double hpa = std::exp(-hh / (47e3 * 1e-6));
		double acc = 0.0;
		for (int j = 0; j < OVER; j++) {
			const double u = noise_gate(hh);
			double x0 = AD[0] * m_x[0] + AD[1] * m_x[1] + AD[2] * m_x[2] + BD[0] * u + BD[1] * m_o8;
			double x1 = AD[3] * m_x[0] + AD[4] * m_x[1] + AD[5] * m_x[2] + BD[2] * u + BD[3] * m_o8;
			double x2 = AD[6] * m_x[0] + AD[7] * m_x[1] + AD[8] * m_x[2] + BD[4] * u + BD[5] * m_o8;
			m_x[0] = x0; m_x[1] = x1; m_x[2] = x2;
			double tgt = std::min(std::max(2.0 * m_x[2], VL), VH);
			m_o8 += std::min(std::max(tgt - m_o8, -SR * hh), SR * hh);
			m_aclp = hpa * m_aclp + (1 - hpa) * m_o8;
			double vt = std::min(std::max(vdc - NOISE_K * (m_o8 - m_aclp), VCLO), VCHI);
			m_vc += std::min(std::max(vt - m_vc, -SR * hh), SR * hh);
			if (m_shi) m_Vc -= KUP * m_vc * hh; else m_Vc += KDN * m_vc * hh;
			const double o = m_vc / 2 - m_Vc;
			if (m_shi && o >= THU) m_shi = false;
			else if (!m_shi && o <= THD) m_shi = true;
			acc += m_shi ? VH : VL;
		}
		const double inv = -0.1 * (acc / OVER - 6.0);
		// Sallen-Key low pass (330 Hz), direct form II transposed
		const double yv = BZ[0] * inv + m_zi[0];
		m_zi[0] = BZ[1] * inv - AZ[1] * yv + m_zi[1];
		m_zi[1] = BZ[2] * inv - AZ[2] * yv;
		// C59 coupling into U15
		const double hp = std::exp(-dt * 2 * 3.14159265358979323846 * HPF);
		m_lpy = hp * m_lpy + (1 - hp) * yv;
		const double gg = vca_gain(m_e2);
		m_t += dt;
		const double xs = (yv - m_lpy) * gg;
		return (soft_clip(xs + BIAS) - soft_clip(BIAS)) * VCLIP;
	}

private:
	// Output stage: soft limiter, x / (1 + |x|^CLIPS)^(1/CLIPS); large CLIPS approaches a hard clip at +-1.
	static double soft_clip(double x)
	{
		const double a = std::fabs(x);
		return x / std::pow(1.0 + std::pow(a, CLIPS), 1.0 / CLIPS);
	}

	// U15 gain versus the control voltage (CY): piecewise linear in dB between knots, zero below the first.
	static double vca_gain(double cy)
	{
		if (cy <= KE[0] - 0.1) return 0.0;
		if (cy < KE[0]) return KG[0] * (cy - (KE[0] - 0.1)) * 10.0;
		for (int i = 1; i < NK; i++)
			if (cy < KE[i]) {
				const double f = (cy - KE[i - 1]) / (KE[i] - KE[i - 1]);
				return KG[i - 1] * std::pow(KG[i] / KG[i - 1], f);
			}
		return KG[NK - 1];
	}

	static constexpr double FCLK = 103200.0;
	GK T1 = 0.1086, T2 = 0.1282;
	GK TAU1 = 0.5249, TAU2 = 0.4812;          // e1 (VCO control) and e2 (VCA control) decay time constants, s
	// U15 model: gain from the table below, then the output stage clips at +-1.
	// Fitted to a recording of the real sound (see README); the recording is hard-clipped in its loud part.
	static constexpr int NK = 7;
	GA KE[NK] = { 2.500, 2.800, 3.200, 3.600, 4.000, 4.400, 4.790 };   // CY (control voltage) knots, V
	GA KG[NK] = { 0.03086, 0.11582, 0.52189, 1.79023, 3.23594, 2.94642, 3.35512 };   // gain at the knots, relative to the clip level (includes the output scaling)
	GK BIAS = 0.150;                                // asymmetry before the limiter (adds even harmonics)
	GK CLIPS = 2.00;                              // output limiter sharpness
	GK VCLIP = 0.5540;                              // volts at SOU for a full-scale (clipped) peak
	GK VH = 10.20, VL = 0.18, THU = 7.46, THD = 3.97;
	GK KUP = 218.2, KDN = 227.3;
	GK VCLO = 0.10, VCHI = 10.20, SR = 0.5e6;
	GK E1_HI = 4.252, E2_HI = 4.79, NOISE_K = 0.405, HPF = 3.0;
	static constexpr double AD[9] = { 8.43322118281427113e-01, 7.73428370356359918e-02, 1.10767148001496545e-03, 2.97472450137061459e-02, 9.44549081654458389e-01, 2.43885633795517462e-02, 7.55230554555658476e-03, 4.32342714455690280e-01, 5.59875822477856833e-01 };
	static constexpr double BD[6] = { 7.82273732029219182e-02, 7.73428370356359779e-02, 1.31510995228375041e-03, -5.54509183455416388e-02, 2.29157520896529376e-04, 4.32342714455690169e-01 };
	static constexpr double BZ[3] = { 4.52803724437418631e-04, 9.05607448874837261e-04, 4.52803724437418631e-04 };
	static constexpr double AZ[3] = { 1.00000000000000000e+00, -1.94080949714154061e+00, 9.42620712039290320e-01 };

	// MM5837: LFSR clock FCLK; returns average gate voltage (4001 inverter: 5 V when pin low) over h seconds
	double noise_gate(double h)
	{
		double rem = h, acc = 0.0;
		while (rem > 1e-15) {
			if (m_bit_left > rem) { acc += m_gate * rem; m_bit_left -= rem; rem = 0; }
			else {
				acc += m_gate * m_bit_left; rem -= m_bit_left; m_bit_left = 1.0 / FCLK;
				const uint32_t t14 = (m_lfsr >> 13) & 1, t17 = (m_lfsr >> 16) & 1;
				m_lfsr = ((m_lfsr << 1) | (t14 ^ t17 ^ (m_lfsr == 0 ? 1 : 0))) & 0x1ffff;
				m_gate = ((m_lfsr >> 16) & 1) ? 0.0 : 5.0;
			}
		}
		return acc / h;
	}

	uint32_t m_lfsr = 0x1ffff;
	double m_bit_left = 1.0 / FCLK, m_gate = 0.0;
	double m_x[3] = { 2.5, -2.5, 2.5 }, m_o8 = 5.0, m_aclp = 5.0, m_vc = 8.4, m_Vc = 0.0;
	bool m_shi = false;
	double m_e1 = 0.0, m_e2 = 0.0, m_t = 0.0, m_ttrig = -1e9;
	double m_zi[2] = { 0.0, 0.0 }, m_lpy = 0.0;
};

} // namespace congo_perc

#undef GK
#undef GA

#endif // CONGO_GORILLA_SYNTH_H
