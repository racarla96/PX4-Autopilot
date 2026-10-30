#!/usr/bin/python3
"""Adds the video scene to a running Gazebo world: a fixed camera on the topic /viewcam and the reference path as dots
(green: start of the trajectory, red: reference every 0.5 s).

    spawn_gz_scene.py --ref ../data/ref_paper2.json [--cam "5 -13 4.4 0 0 1.5708"] [--fov 0.95]

Run it with the system python3 (the Gazebo bindings are installed there, not in a virtualenv).
"""
import argparse
import json
import subprocess

ap = argparse.ArgumentParser()
ap.add_argument("--ref", required=True)
ap.add_argument("--cam", default="5 -13 4.4 0 0 1.5708", help="camera pose in the ENU world: x y z roll pitch yaw")
ap.add_argument("--fov", type=float, default=0.95)
ap.add_argument("--size", default="1920x1080")
a = ap.parse_args()


def create(sdf):
    r = subprocess.run(["gz", "service", "-s", "/world/default/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "5000", "--req", 'sdf: "%s"' % sdf.replace('"', '\\"').replace("\n", " ")], capture_output=True, text=True)
    return "true" in r.stdout


w, h = a.size.split("x")
cam = f"""<sdf version="1.9"><model name="viewcam"><static>true</static><pose>{a.cam}</pose><link name="l"><sensor name="cam" type="camera">
<camera><horizontal_fov>{a.fov}</horizontal_fov><image><width>{w}</width><height>{h}</height></image><clip><near>0.1</near><far>300</far></clip></camera>
<always_on>1</always_on><update_rate>20</update_rate><visualize>false</visualize><topic>viewcam</topic></sensor></link></model></sdf>"""
print("camera", create(cam))
ref = json.load(open(a.ref))
for i, (x, y, z) in enumerate(ref):  # NED -> ENU
    rad = 0.09 if i in (0, len(ref) - 1) else 0.035
    col = "0 0.6 0 1" if i == 0 else "1 0 0 1"
    create(f'<sdf version="1.9"><model name="ref{i}"><static>true</static><pose>{y} {x} {-z} 0 0 0</pose><link name="l"><visual name="v"><geometry>'
           f"<sphere><radius>{rad}</radius></sphere></geometry><material><ambient>{col}</ambient><diffuse>{col}</diffuse><emissive>{col}</emissive>"
           "</material></visual></link></model></sdf>")
print("dots", len(ref))
