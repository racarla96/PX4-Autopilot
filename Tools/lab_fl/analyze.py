#!/usr/bin/env python3
"""Analysis of the LAB / FL flights: tracking metrics and comparison plots from PX4 logs (.ulg).

    analyze.py [--out DIR] run1.ulg run2.ulg ...

The label of a run is the file name without extension. If it has the form <trajectory>_<controller>_<gains>
(what experiment.sh writes, e.g. paper2_lab_robust.ulg) the runs are grouped by trajectory and one set of plots is
drawn per trajectory. For every run the table gives the tracking error while the mode is active (RMSE per axis, 3D
RMSE and maximum, yaw RMSE), the control effort (RMS of the commanded torques, standard deviation of the thrust),
the maximum tilt, the fraction of time with saturation and the settling time after the end of a finite trajectory.

Needs: pyulog, numpy, matplotlib.
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pyulog import ULog  # noqa: E402

TOPICS = ["traj_control_status", "trajectory_reference", "vehicle_attitude", "vehicle_thrust_setpoint",
          "vehicle_torque_setpoint"]
COLORS = {"lab": "tab:blue", "fl": "tab:red"}
STYLES = {"robust": "-", "paper1": "--", "paper2": "--"}
SETTLE_BAND = 0.10  # m


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def vec(d, name, n=3, mask=None):
    m = np.column_stack([d["%s[%d]" % (name, i)] for i in range(n)])
    return m if mask is None else m[mask]


def load(path):
    log = ULog(path, message_name_filter_list=TOPICS)
    data = {d.name: d.data for d in log.data_list}
    st = data["traj_control_status"]
    active = st["active"] > 0
    if active.any():
        # the last 0.5 s are dropped: the reference topic stops with the mode and its last samples are not meaningful
        active &= st["timestamp"] <= st["timestamp"][active][-1] - 500000
    if not active.any():
        raise RuntimeError("%s: the trajectory controller was never active" % path)
    t_abs = st["timestamp"][active] / 1e6
    t0 = t_abs[0]
    run = {"t": t_abs - t0}
    run["controller"] = int(st["controller"][active][0])
    run["ref"] = vec(st, "position_ref", mask=active)
    run["pos"] = vec(st, "position", mask=active)
    run["err"] = vec(st, "position_error", mask=active)
    run["thrust_status"] = st["thrust"][active]
    for k in ("thrust_saturated", "torque_saturated", "tilt_saturated"):
        run[k] = st[k][active] > 0

    att = data["vehicle_attitude"]
    q = np.column_stack([att["q[%d]" % i] for i in range(4)])
    ta = att["timestamp"] / 1e6 - t0
    run["yaw"] = np.interp(run["t"], ta, np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                                                1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))
    run["tilt_t"] = ta
    run["tilt"] = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))

    ref = data["trajectory_reference"]
    tr = ref["timestamp"] / 1e6 - t0
    run["yaw_ref"] = np.interp(run["t"], tr, np.unwrap(ref["yaw"]))
    run["yaw_err"] = wrap(run["yaw_ref"] - run["yaw"])
    fin = np.where(ref["finished"] > 0)[0]
    run["t_finished"] = float(tr[fin[0]]) if len(fin) else float("nan")

    for name, key in (("vehicle_torque_setpoint", "torque"), ("vehicle_thrust_setpoint", "thrust_sp")):
        if name in data:
            d = data[name]
            tt = d["timestamp"] / 1e6 - t0
            m = (tt >= 0) & (tt <= run["t"][-1])
            run[key + "_t"] = tt[m]
            if key == "torque":
                run[key] = np.column_stack([d["xyz[%d]" % i][m] for i in range(3)])
            else:
                run[key] = -d["xyz[2]"][m]  # thrust along -z of the body frame
    return run


def metrics(run):
    e = run["err"]
    e3 = np.linalg.norm(e, axis=1)
    rms = np.sqrt((e ** 2).mean(axis=0))
    m = {
        "rmse_x [m]": rms[0], "rmse_y [m]": rms[1], "rmse_z [m]": rms[2],
        "rmse_3d [m]": float(np.sqrt((e3 ** 2).mean())), "max_3d [m]": float(e3.max()),
        "rmse_yaw [deg]": float(np.degrees(np.sqrt((run["yaw_err"] ** 2).mean()))),
        "max_yaw [deg]": float(np.degrees(np.abs(run["yaw_err"]).max())),
    }
    if "torque" in run:
        tq = np.sqrt((run["torque"] ** 2).mean(axis=0))
        m.update({"rms_torque_roll": tq[0], "rms_torque_pitch": tq[1], "rms_torque_yaw": tq[2]})
    if "thrust_sp" in run:
        m["thrust_std"] = float(run["thrust_sp"].std())
    m["max_tilt [deg]"] = float(run["tilt"][(run["tilt_t"] >= 0) & (run["tilt_t"] <= run["t"][-1])].max())
    m["saturated [%]"] = 100.0 * float((run["thrust_saturated"] | run["torque_saturated"] | run["tilt_saturated"]).mean())
    tf = run["t_finished"]
    settle = float("nan")
    if np.isfinite(tf):
        after = run["t"] >= tf
        out = np.where(after & (e3 > SETTLE_BAND))[0]
        settle = float(run["t"][out[-1]] - tf) if len(out) else 0.0
        if len(out) and out[-1] == len(e3) - 1:
            settle = float("inf")  # never settled within the log
    m["settling [s]"] = settle
    m["duration [s]"] = float(run["t"][-1])
    return m


def label_parts(label):
    parts = label.split("_")
    if len(parts) >= 3:
        return parts[0], parts[1], parts[2]
    return label, "", ""


def plot_group(traj, runs, out):
    fig, ax = plt.subplots(figsize=(7, 6))
    ref = next(iter(runs.values()))["ref"]
    ax.plot(ref[:, 1], ref[:, 0], "k:", lw=1.5, label="reference")
    for label, r in runs.items():
        _, ctrl, gains = label_parts(label)
        ax.plot(r["pos"][:, 1], r["pos"][:, 0], color=COLORS.get(ctrl), ls=STYLES.get(gains, "-"), label=label)
    ax.set_xlabel("East y [m]")
    ax.set_ylabel("North x [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("%s: horizontal path" % traj)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "%s_path.png" % traj), dpi=130)
    plt.close(fig)

    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    first = next(iter(runs.values()))
    for i, name in enumerate(("x [m]", "y [m]", "z [m] (down)")):
        axs[i].plot(first["t"], first["ref"][:, i], "k:", lw=1.5, label="reference")
    axs[3].plot(first["t"], np.degrees(first["yaw_ref"]), "k:", lw=1.5, label="reference")
    for label, r in runs.items():
        _, ctrl, gains = label_parts(label)
        kw = dict(color=COLORS.get(ctrl), ls=STYLES.get(gains, "-"), label=label)
        for i in range(3):
            axs[i].plot(r["t"], r["pos"][:, i], **kw)
        axs[3].plot(r["t"], np.degrees(r["yaw"]), **kw)
    for a, y in zip(axs, ("x [m]", "y [m]", "z [m] (down)", "yaw [deg]")):
        a.set_ylabel(y)
        a.grid(alpha=0.3)
    axs[0].legend(fontsize=8, ncol=2)
    axs[3].set_xlabel("t [s]")
    fig.suptitle("%s: position and yaw against the reference" % traj)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "%s_timeseries.png" % traj), dpi=130)
    plt.close(fig)

    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for label, r in runs.items():
        _, ctrl, gains = label_parts(label)
        kw = dict(color=COLORS.get(ctrl), ls=STYLES.get(gains, "-"), label=label)
        axs[0].plot(r["t"], np.linalg.norm(r["err"], axis=1), **kw)
        axs[1].plot(r["t"], np.degrees(np.abs(r["yaw_err"])), **kw)
        m = (r["tilt_t"] >= 0) & (r["tilt_t"] <= r["t"][-1])
        axs[2].plot(r["tilt_t"][m], r["tilt"][m], **kw)
        if "torque" in r:
            axs[3].plot(r["torque_t"], np.linalg.norm(r["torque"], axis=1), **kw)
    for a, y in zip(axs, ("position error 3D [m]", "|yaw error| [deg]", "tilt [deg]", "|torque setpoint| (normalized)")):
        a.set_ylabel(y)
        a.grid(alpha=0.3)
    axs[0].legend(fontsize=8, ncol=2)
    axs[3].set_xlabel("t [s]")
    fig.suptitle("%s: errors, tilt and control effort" % traj)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "%s_errors.png" % traj), dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default=".", help="directory for the plots and metrics.csv")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    groups = defaultdict(dict)
    rows = []
    for path in args.logs:
        label = os.path.splitext(os.path.basename(path))[0]
        try:
            run = load(path)
        except Exception as exc:  # keep going with the other logs
            print("skipped %s: %s" % (path, exc), file=sys.stderr)
            continue
        m = metrics(run)
        rows.append((label, m))
        groups[label_parts(label)[0]][label] = run

    if not rows:
        sys.exit("no usable log")

    cols = list(rows[0][1].keys())
    with open(os.path.join(args.out, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run"] + cols)
        for label, m in rows:
            w.writerow([label] + ["%.4g" % m.get(c, float("nan")) for c in cols])

    width = max(len(label) for label, _ in rows)
    print("%-*s " % (width, "run") + " ".join("%14s" % c[:14] for c in cols))
    for label, m in rows:
        print("%-*s " % (width, label) + " ".join("%14.3f" % m.get(c, float("nan")) for c in cols))

    for traj, runs in groups.items():
        plot_group(traj, runs, args.out)
    print("plots and metrics.csv written to %s" % args.out)


if __name__ == "__main__":
    main()
