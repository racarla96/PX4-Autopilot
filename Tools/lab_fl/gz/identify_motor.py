#!/usr/bin/env python3
"""Fits the motor + propeller model of the Gazebo drone from a thrust-bench sweep (RCbenchmark CSV).

    identify_motor.py data/motor_1806_4045_4S.csv [--pwm-min 1030 --pwm-max 1900] [--out motor.json]

Model (same as the identification of the TFE, identificacion_motor1806.m):
    thrust = kT * w^2            [N, w in rad/s]
    torque = kQ * w^2            [N m]
and, for the simulator, the rotor speed as a linear function of the normalized PX4 output u in [0, 1]:
    w(u) = ec_min + (ec_max - ec_min) * u,     u = (pwm - pwm_min) / (pwm_max - pwm_min)
fitted on the thrust range that matters in flight (u >= FIT_U_MIN), because the Gazebo plugin only takes a linear map
(SIM_GZ_EC_MINx / SIM_GZ_EC_MAXx). `thr_mdl_fac` is the PX4 thrust-model factor (THR_MDL_FAC) that best linearizes the
measured thrust over the same range: thrust/thrust_max = (1 - f) u + f u^2.

The ESC of the bench saturates its duty cycle near 92 % (the last PWM steps do not increase the speed), so pwm_max
should be the PWM where the sweep stops growing (1900 us for the 1806).
"""
import argparse
import csv
import json
import math

import numpy as np

G0 = 9.80665
FIT_U_MIN = 0.4
RPM_MIN_VALID = 200


def load(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    col = lambda name: np.array([float(r[name]) if r[name] not in ("", None) else math.nan for r in rows])
    return {
        "pwm": col("ESC signal (µs)"), "thrust_n": col("Thrust (kgf)") * G0, "torque": col("Torque (N·m)"),
        "rpm": col("Motor Optical Speed (RPM)"), "volt": col("Voltage (V)"), "amp": col("Current (A)"),
    }


def fit(path, pwm_min=1030, pwm_max=1900):
    """Returns the dictionary of the fitted model (see the module docstring)."""
    d = load(path)
    ok = d["rpm"] > RPM_MIN_VALID
    for k in d:
        d[k] = d[k][ok]
    w = d["rpm"] * 2 * math.pi / 60
    kT = float((d["thrust_n"] * w ** 2).sum() / (w ** 4).sum())
    kQ = float((d["torque"] * w ** 2).sum() / (w ** 4).sum())
    t_fit = kT * w ** 2
    rel_err = float(np.sqrt(np.mean((t_fit - d["thrust_n"]) ** 2)) / d["thrust_n"].max())

    # speed against the normalized output, linear fit over the flight range
    u = np.clip((d["pwm"] - pwm_min) / (pwm_max - pwm_min), 0, 1.2)
    sel = (u >= FIT_U_MIN) & (u <= 1.0)
    slope, icpt = np.polyfit(u[sel], w[sel], 1)
    ec_min, ec_max = float(icpt), float(icpt + slope)
    w_sim = icpt + slope * u
    # thrust-model factor over the same range
    tmax = d["thrust_n"][u <= 1.0].max()
    tn = d["thrust_n"][sel] / tmax
    us = u[sel]
    # tn = (1-f) us + f us^2  ->  tn - us = f (us^2 - us)
    f = float(np.clip(((tn - us) * (us ** 2 - us)).sum() / ((us ** 2 - us) ** 2).sum(), 0, 1))

    res = {
        "csv": path, "kT": kT, "kQ": kQ, "moment_constant": kQ / kT, "fit_thrust_rms_error_rel": rel_err,
        "pwm_min": pwm_min, "pwm_max": pwm_max, "ec_min": ec_min, "ec_max": ec_max,
        "omega_max": float(w[u <= 1.0].max()), "thrust_max_n": float(tmax),
        "thr_mdl_fac": f,
        "sim_speed_error_rel_max": float(np.max(np.abs(w_sim[sel] - w[sel]) / w[sel])),
        "voltage_v": [float(d["volt"].max()), float(d["volt"].min())], "current_max_a": float(d["amp"].max()),
        "thrust_n_at_u": {"%.1f" % x: float(np.interp(x, u, d["thrust_n"])) for x in (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)},
    }
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--pwm-min", type=float, default=1030, help="lowest PWM where the motor spins (PX4 output 0)")
    ap.add_argument("--pwm-max", type=float, default=1900, help="PWM where the speed stops growing (PX4 output 1)")
    ap.add_argument("--out", default=None, help="write the result as JSON")
    a = ap.parse_args()
    res = fit(a.csv, a.pwm_min, a.pwm_max)
    for k, v in res.items():
        print("%-26s %s" % (k, v))
    if a.out:
        with open(a.out, "w") as f_:
            json.dump(res, f_, indent=2)


if __name__ == "__main__":
    main()
