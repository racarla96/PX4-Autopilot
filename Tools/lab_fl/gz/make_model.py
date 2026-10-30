#!/usr/bin/env python3
"""Builds the Gazebo model, the SITL airframe and the parameter file of the lab drone from zmr250.toml.

    make_model.py                 # reads zmr250.toml, the meshes of models/zmr250/meshes and the bench CSV
    make_model.py --check         # only prints the mass report

Writes
    models/zmr250/model.sdf, model.config          Gazebo model (include with model://zmr250)
    ../../../ROMFS/px4fmu_common/init.d-posix/airframes/4050_gz_zmr250   SITL airframe (geometry, motors, TCC_*)
    params/zmr250.params                           the same PX4 parameters as a QGroundControl file (real drone)
    mass_report.md                                 mass, centre of mass, inertia and thrust margins

Mass, centre of mass and inertia come from the bodies of the TOML: a mesh spreads its mass over its volume, a box is
uniform, a point list is a set of point masses. The propellers and a share of each motor spin: they are separate links of
the SDF; the rest is the base link. PX4 wants the rotor positions relative to the centre of mass, they are computed here.
"""
import argparse
import json
import math
import os
import struct
import sys
import tomllib

import numpy as np

import identify_motor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../.."))
MODEL_DIR = os.path.join(HERE, "models", "zmr250")
AIRFRAME = os.path.join(ROOT, "ROMFS/px4fmu_common/init.d-posix/airframes/4050_gz_zmr250")
G = 9.80665


# ---------------------------------------------------------------- mass properties
def load_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def mesh_props(tri):
    """Volume, centre of mass and inertia about the centre of mass (unit density) of a closed triangle mesh."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6
    com = (det[:, None] * (a + b + c)).sum(0) / 24 / vol
    s = a + b + c
    second = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            second[i, j] = (det / 120 * (s[:, i] * s[:, j] + a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])).sum()
    inertia_o = np.trace(second) * np.eye(3) - second
    d = com
    inertia = inertia_o - vol * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return vol, com, inertia


def box_inertia(m, size):
    x, y, z = size
    return np.diag([y * y + z * z, x * x + z * z, x * x + y * y]) * m / 12


def cyl_inertia(m, r, h):
    return np.diag([m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2])


def bar_inertia(m, length, width, thick):
    return np.diag([m * (width ** 2 + thick ** 2) / 12, m * (length ** 2 + thick ** 2) / 12, m * (length ** 2 + width ** 2) / 12])


class Piece:
    def __init__(self, name, mass, com, inertia, status=""):
        self.name, self.mass, self.com, self.inertia, self.status = name, mass, np.array(com, float), np.array(inertia, float), status


def combine(pieces):
    m = sum(p.mass for p in pieces)
    com = sum(p.mass * p.com for p in pieces) / m
    inertia = np.zeros((3, 3))
    for p in pieces:
        d = p.com - com
        inertia += p.inertia + p.mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, inertia


def rot_z_pi(tri):
    out = tri.copy()
    out[..., 0] *= -1
    out[..., 1] *= -1
    return out


# ---------------------------------------------------------------- build
def build(cfg):
    mt, pr, lg = cfg["motor"], cfg["propeller"], cfg["legs"]
    pieces_static, pieces_rotor = [], []
    mesh_cache = {}
    notes = []

    for b in cfg["body"]:
        if b["kind"] == "mesh":
            props = os.path.join(HERE, "data", "%s_cad_props.json" % b["name"])
            if os.path.exists(props):  # exact CAD properties (convert_cad.py); the decimated STL does not keep the volume
                p = json.load(open(props))
                vol, com = p["volume_m3"], np.array(p["com_m"])
                pieces_static.append(Piece(b["name"], b["mass"], com, np.array(p["inertia_per_mass_m2"]) * b["mass"], b.get("status", "")))
                notes.append("%s: exact CAD volume %.1f cm3 -> mean density %.0f kg/m3" % (b["name"], vol * 1e6, b["mass"] / vol))
            else:
                tri = load_stl(os.path.join(MODEL_DIR, "meshes", b["mesh"]))
                vol, com, i_unit = mesh_props(tri)
                pieces_static.append(Piece(b["name"], b["mass"], com, i_unit * b["mass"] / vol, b.get("status", "")))
                notes.append("%s: mesh volume %.1f cm3 -> mean density %.0f kg/m3" % (b["name"], vol * 1e6, b["mass"] / vol))
        elif b["kind"] == "box":
            pieces_static.append(Piece(b["name"], b["mass"], b["pose"], box_inertia(b["mass"], b["size"]), b.get("status", "")))
        elif b["kind"] == "points":
            n = len(b["points"])
            for k, p in enumerate(b["points"]):
                pieces_static.append(Piece("%s_%d" % (b["name"], k), b["mass"] / n, p, np.zeros((3, 3)), b.get("status", "")))

    # printed legs: rear mesh and its copy turned 180 degrees about z
    tri = load_stl(os.path.join(MODEL_DIR, "meshes", "leg_rear.stl"))
    vol, com, i_unit = mesh_props(tri)
    leg_mass = vol * lg["density_asa"] * lg["infill_factor"]
    for k, flip in enumerate((False, True)):
        c = com.copy()
        i_m = i_unit * leg_mass / vol
        if flip:
            c[0], c[1] = -c[0], -c[1]
        pieces_static.append(Piece("leg_%s" % ("front" if flip else "rear"), leg_mass, c, i_m, "estimate"))
    notes.append("leg: %.1f cm3 -> %.1f g each (ASA %.0f kg/m3, infill factor %.2f)" % (vol * 1e6, leg_mass * 1e3, lg["density_asa"], lg["infill_factor"]))

    # motors and propellers
    motor_pos = {0: (mt["x"], -mt["y"]), 1: (-mt["x"], mt["y"]), 2: (mt["x"], mt["y"]), 3: (-mt["x"], -mt["y"])}
    spin = {0: "ccw", 1: "ccw", 2: "cw", 3: "cw"}
    rotor_mass = pr["mass"] + mt["rotor_mass_fraction"] * mt["mass"]
    static_motor_mass = (1 - mt["rotor_mass_fraction"]) * mt["mass"]
    hub = np.array([0, 0, pr["z"]])
    rotor_inertia = bar_inertia(pr["mass"], pr["diameter"], pr["chord"], pr["thickness"]) + cyl_inertia(
        mt["rotor_mass_fraction"] * mt["mass"], mt["radius"] * 0.9, mt["height"] * 0.5)
    for i, (x, y) in motor_pos.items():
        pieces_static.append(Piece("motor_%d_stator" % i, static_motor_mass,
                                   [x, y, mt["base_z"] + mt["height"] / 2],
                                   cyl_inertia(static_motor_mass, mt["radius"], mt["height"]), "estimate"))
        pieces_rotor.append(Piece("rotor_%d" % i, rotor_mass, [x, y, pr["z"]], rotor_inertia, "measured/estimate"))
    return pieces_static, pieces_rotor, motor_pos, spin, rotor_mass, rotor_inertia, notes


def fmt(v, n=9):
    return ("%." + str(n) + "g") % v


def inertial_xml(mass, com, inertia, indent):
    pad = " " * indent
    return (f"{pad}<inertial>\n{pad}  <pose>{fmt(com[0])} {fmt(com[1])} {fmt(com[2])} 0 0 0</pose>\n{pad}  <mass>{fmt(mass)}</mass>\n"
            f"{pad}  <inertia>\n{pad}    <ixx>{fmt(inertia[0, 0])}</ixx>\n{pad}    <ixy>{fmt(inertia[0, 1])}</ixy>\n"
            f"{pad}    <ixz>{fmt(inertia[0, 2])}</ixz>\n{pad}    <iyy>{fmt(inertia[1, 1])}</iyy>\n"
            f"{pad}    <iyz>{fmt(inertia[1, 2])}</iyz>\n{pad}    <izz>{fmt(inertia[2, 2])}</izz>\n{pad}  </inertia>\n{pad}</inertial>\n")


def write_sdf(cfg, static, rotors, motor_pos, spin, rotor_mass, rotor_inertia, motor, com_total):
    mt, pr = cfg["motor"], cfg["propeller"]
    m_s, com_s, i_s = combine(static)
    sensors = open(os.path.join(HERE, "templates/sensors.sdf")).read()
    bodies = {b["name"]: b for b in cfg["body"]}

    def mesh_vis(name, mesh, pose, scale=1.0, color=None):
        mat = ""
        if color:
            mat = f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
        return (f'      <visual name="{name}">\n        <pose>{pose}</pose>\n        <geometry><mesh><scale>{scale} {scale} {scale}</scale>'
                f'<uri>model://zmr250/meshes/{mesh}</uri></mesh></geometry>\n        {mat}\n      </visual>\n')

    def box_vis(name, size, pose, color):
        return (f'      <visual name="{name}">\n        <pose>{pose[0]} {pose[1]} {pose[2]} 0 0 0</pose>\n        <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>\n'
                f'        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>\n      </visual>\n')

    out = ['<?xml version="1.0" encoding="UTF-8"?>\n<!-- Generated by Tools/lab_fl/gz/make_model.py from zmr250.toml. Do not edit by hand. -->\n',
           "<sdf version='1.9'>\n  <model name='zmr250'>\n    <pose>0 0 %s 0 0 0</pose>\n    <self_collide>false</self_collide>\n    <static>false</static>\n" % cfg["vehicle"]["spawn_z"],
           '    <link name="base_link">\n      <gravity>true</gravity>\n      <velocity_decay />\n']
    out.append(inertial_xml(m_s, com_s, i_s, 6))
    out.append(mesh_vis("frame_visual", "frame.stl", "0 0 0 0 0 0", color="0.12 0.12 0.12 1"))
    out.append(mesh_vis("leg_rear_visual", "leg_rear.stl", "0 0 0 0 0 0", color="0.9 0.5 0.1 1"))
    out.append(mesh_vis("leg_front_visual", "leg_rear.stl", "0 0 0 0 0 3.141592654", color="0.9 0.5 0.1 1"))
    p = bodies["pixhawk"]["pose"]
    out.append(mesh_vis("pixhawk_visual", "pixhawk.stl", f"{p[0]} {p[1]} {p[2]} 0 0 0", color="0.3 0.3 0.35 1"))
    b = bodies["battery"]
    out.append(box_vis("battery_visual", b["size"], b["pose"], "0.1 0.2 0.6 1"))
    b = bodies["beacon"]
    if b.get("visual_mesh"):  # CAD of the beacon; its bottom face sits on the pose below, centred on x, y
        vp = b["visual_pose"]
        out.append(mesh_vis("beacon_visual", b["visual_mesh"], f"{vp[0]} {vp[1]} {vp[2]} 0 0 0", color="0.1 0.1 0.12 1"))
    else:
        out.append(box_vis("beacon_visual", b["size"], b["pose"], "0.1 0.7 0.2 1"))
    for i, (x, y) in motor_pos.items():
        out.append(mesh_vis("motor_%d_visual" % i, "motor.stl", f"{x} {y} {mt['base_z']} 0 0 0", color="0.6 0.6 0.65 1"))
    # contact: the two legs (bottom 8 mm of each bar), the only parts that touch the ground
    leg = load_stl(os.path.join(MODEL_DIR, "meshes", "leg_rear.stl")).reshape(-1, 3)
    lo, hi = leg.min(0), leg.max(0)
    size = [hi[0] - lo[0], hi[1] - lo[1], 0.008]
    for name, sx in (("rear", 1), ("front", -1)):
        cx = sx * (lo[0] + hi[0]) / 2
        out.append(f'      <collision name="leg_{name}_collision">\n        <pose>{fmt(cx)} 0 {fmt(lo[2] + 0.004)} 0 0 0</pose>\n'
                   f'        <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>\n'
                   '        <surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode /></friction></surface>\n      </collision>\n')
    # sensors at the centre of mass of the vehicle (PX4 assumes the IMU there)
    out.append(f"      <pose>{fmt(com_total[0])} {fmt(com_total[1])} {fmt(com_total[2])} 0 0 0</pose>\n" if False else "")
    out.append(sensors)
    cam = cfg.get("camera", {})
    if cam.get("enabled"):
        # camera fixed to the vehicle that looks at it from behind and above: a chase view for videos (off by default)
        x, z, pitch = cam["x"], cam["z"], cam["pitch"]
        out.append(f'      <sensor name="chase_cam" type="camera">\n        <pose>{x} 0 {z} 0 {pitch} 0</pose>\n        <camera><horizontal_fov>{cam["fov"]}</horizontal_fov>'
                   f'<image><width>{cam["width"]}</width><height>{cam["height"]}</height></image><clip><near>0.05</near><far>300</far></clip></camera>\n'
                   '        <always_on>1</always_on>\n        <update_rate>20</update_rate>\n        <visualize>false</visualize>\n        <topic>chase_cam</topic>\n      </sensor>\n')
    out.append("    </link>\n")
    for i, (x, y) in motor_pos.items():
        out.append(f'    <link name="rotor_{i}">\n      <gravity>true</gravity>\n      <self_collide>false</self_collide>\n      <velocity_decay />\n      <pose>{x} {y} {pr["z"]} 0 0 0</pose>\n')
        out.append(inertial_xml(rotor_mass, [0, 0, 0], rotor_inertia, 6))
        out.append(mesh_vis("rotor_%d_visual" % i, "prop.stl", "0 0 0 0 0 0", color="0.05 0.05 0.05 1"))
        out.append("    </link>\n")
        out.append(f'    <joint name="rotor_{i}_joint" type="revolute">\n      <parent>base_link</parent>\n      <child>rotor_{i}</child>\n'
                   '      <axis>\n        <xyz>0 0 1</xyz>\n        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>\n'
                   '        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>\n      </axis>\n    </joint>\n')
    # motor model: thrust = motorConstant * w^2, moment = momentConstant * thrust
    scale = motor["kT"] / 8.54858e-06  # the x500 plugin values are scaled with the thrust coefficient
    for i in motor_pos:
        out.append(f'''    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{spin[i]}</turningDirection>
      <timeConstantUp>{mt["time_constant"]}</timeConstantUp>
      <timeConstantDown>{mt["time_constant"]}</timeConstantDown>
      <maxRotVelocity>{fmt(motor["ec_max"] * 1.05, 5)}</maxRotVelocity>
      <motorConstant>{fmt(motor["kT"])}</motorConstant>
      <momentConstant>{fmt(motor["moment_constant"])}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>{fmt(8.06428e-05 * scale)}</rotorDragCoefficient>
      <rollingMomentCoefficient>{fmt(1e-06 * scale)}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>30</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
''')
    out.append("  </model>\n</sdf>\n")
    open(os.path.join(MODEL_DIR, "model.sdf"), "w").write("".join(out))
    open(os.path.join(MODEL_DIR, "model.config"), "w").write(
        '<?xml version="1.0"?>\n<model>\n  <name>zmr250</name>\n  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
        '  <description>Lab drone: ZMR250 frame, 4 x 1806 motors, 4045 propellers, Pixhawk 2.4.8 (generated, see Tools/lab_fl/gz)</description>\n</model>\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "zmr250.toml"))
    ap.add_argument("--check", action="store_true", help="print the mass report only")
    ap.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE",
                    help="override a value of the TOML for this run, e.g. --set motor.time_constant=0.02 (repeatable)")
    a = ap.parse_args()
    cfg = tomllib.load(open(a.config, "rb"))
    for item in a.set:
        key, val = item.split("=", 1)
        section, name = key.split(".", 1)
        cfg[section][name] = tomllib.loads("v = " + val)["v"]
    mt = cfg["motor"]

    # motor model from the bench data
    csv = os.path.join(HERE, mt["bench_csv"])
    motor = identify_motor.fit(csv, mt["pwm_min"], mt["pwm_max"])

    static, rotors, motor_pos, spin, rotor_mass, rotor_inertia, notes = build(cfg)
    m_tot, com, inertia = combine(static + rotors)
    t_rotor_max = motor["kT"] * motor["ec_max"] ** 2
    t_total_max = 4 * t_rotor_max
    hover = m_tot * G / t_total_max
    px = {i: (x - com[0]) for i, (x, y) in motor_pos.items()}              # forward
    py = {i: -(y - com[1]) for i, (x, y) in motor_pos.items()}             # right
    pz = {i: -(cfg["propeller"]["z"] - com[2]) for i in motor_pos}         # down
    lever = min(abs(v) for v in list(px.values()) + list(py.values()))
    tq_xy = 2 * math.sqrt(2) * lever * t_rotor_max
    tq_z = 4 * motor["moment_constant"] * t_rotor_max

    rep = ["# ZMR250 mass report", "", "Generated by `make_model.py` from `zmr250.toml`. Positions in the body frame of the model (x forward, y left, z up).", "",
           "| body | mass [g] | status |", "|---|---|---|"]
    for p in static + rotors:
        if p.name.startswith(("esc_", "motor_", "rotor_")):
            continue
        rep.append("| %s | %.1f | %s |" % (p.name, p.mass * 1e3, p.status))
    rep.append("| 4 x motor (stator + rotor) | %.1f | estimate |" % (4 * cfg["motor"]["mass"] * 1e3))
    rep.append("| 4 x propeller | %.1f | measured |" % (4 * cfg["propeller"]["mass"] * 1e3))
    rep += ["", "| quantity | value |", "|---|---|",
            "| total mass | %.1f g |" % (m_tot * 1e3),
            "| centre of mass (x, y, z) | %.1f, %.1f, %.1f mm |" % tuple(com * 1e3),
            "| inertia Ixx, Iyy, Izz about the centre of mass | %.3e, %.3e, %.3e kg m^2 |" % (inertia[0, 0], inertia[1, 1], inertia[2, 2]),
            "| products of inertia Ixy, Ixz, Iyz | %.1e, %.1e, %.1e kg m^2 |" % (inertia[0, 1], inertia[0, 2], inertia[1, 2]),
            "| max thrust per rotor (model, output 1) | %.2f N |" % t_rotor_max,
            "| max thrust / weight | %.2f |" % (t_total_max / (m_tot * G)),
            "| hover thrust, share of the maximum | %.2f |" % hover,
            "| hover thrust per rotor | %.2f N |" % (m_tot * G / 4),
            "| TCC_TQ_MAX_XY (2 sqrt(2) min(arm) T_max) | %.3f N m |" % tq_xy,
            "| TCC_TQ_MAX_Z (4 KM T_max) | %.3f N m |" % tq_z, "",
            "Motor model (identify_motor.py on `%s`): kT %.4e N/(rad/s)^2, moment constant %.4f m, speed %.0f + %.0f u rad/s, THR_MDL_FAC %.2f."
            % (mt["bench_csv"], motor["kT"], motor["moment_constant"], motor["ec_min"], motor["ec_max"] - motor["ec_min"], motor["thr_mdl_fac"]), "",
            "Notes:"] + ["- " + n for n in notes]
    open(os.path.join(HERE, "mass_report.md"), "w").write("\n".join(rep) + "\n")
    print("\n".join(rep))
    if a.check:
        return

    write_sdf(cfg, static, rotors, motor_pos, spin, rotor_mass, rotor_inertia, motor, com)

    ca = []
    for i in motor_pos:
        km = motor["moment_constant"] * (1 if spin[i] == "ccw" else -1)
        ca += ["CA_ROTOR%d_PX %.5f" % (i, px[i]), "CA_ROTOR%d_PY %.5f" % (i, py[i]), "CA_ROTOR%d_PZ %.5f" % (i, pz[i]), "CA_ROTOR%d_KM %.5f" % (i, km)]
    vehicle = ["CA_AIRFRAME 0", "CA_ROTOR_COUNT 4"] + ca + ["THR_MDL_FAC %.2f" % motor["thr_mdl_fac"], "MPC_THR_HOVER %.2f" % hover,
              "TCC_MASS %.4f" % m_tot, "TCC_IXX %.6f" % inertia[0, 0], "TCC_IYY %.6f" % inertia[1, 1], "TCC_IZZ %.6f" % inertia[2, 2],
              "TCC_TQ_MAX_XY %.4f" % tq_xy, "TCC_TQ_MAX_Z %.4f" % tq_z]
    sim = ["SIM_GZ_EN 1"]
    for k in range(4):
        sim += ["SIM_GZ_EC_FUNC%d %d" % (k + 1, 101 + k), "SIM_GZ_EC_MIN%d %.0f" % (k + 1, motor["ec_min"]), "SIM_GZ_EC_MAX%d %.0f" % (k + 1, motor["ec_max"])]
    with open(AIRFRAME, "w") as f:
        f.write("#!/bin/sh\n#\n# @name Gazebo ZMR250 (lab drone)\n#\n# @type Quadrotor\n#\n"
                "# Generated by Tools/lab_fl/gz/make_model.py from zmr250.toml. Do not edit by hand.\n\n"
                ". ${R}etc/init.d/rc.mc_defaults\n\nPX4_SIMULATOR=${PX4_SIMULATOR:=gz}\nPX4_GZ_WORLD=${PX4_GZ_WORLD:=default}\nPX4_SIM_MODEL=${PX4_SIM_MODEL:=zmr250}\n\n")
        for line in sim + vehicle:
            f.write("param set-default %s\n" % line)
    os.makedirs(os.path.join(HERE, "params"), exist_ok=True)
    with open(os.path.join(HERE, "params/zmr250.params"), "w") as f:
        f.write("# PX4 parameters of the lab drone (QGroundControl format: vehicle, component, name, value, type)\n"
                "# Generated by make_model.py. Load them in QGC (Parameters > Tools > Load from file).\n")
        f.write("# The PWM range of the real ESC is not set here: PWM_MAIN_MIN %d, PWM_MAIN_MAX %d must match the bench (%d..%d us).\n" % (
            mt["pwm_min"], mt["pwm_max"], mt["pwm_min"], mt["pwm_max"]))
        for line in vehicle:
            name, val = line.split()
            typ = 6 if name in ("CA_AIRFRAME", "CA_ROTOR_COUNT") else 9
            f.write("1\t1\t%s\t%s\t%d\n" % (name, val, typ))
    print("wrote", os.path.join(MODEL_DIR, "model.sdf"), AIRFRAME, os.path.join(HERE, "params/zmr250.params"))


if __name__ == "__main__":
    main()
