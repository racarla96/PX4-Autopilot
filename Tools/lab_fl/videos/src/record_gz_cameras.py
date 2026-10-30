#!/usr/bin/python3
"""Records camera topics of Gazebo to MP4 files, one frame per simulation frame (the cameras run at 20 Hz of simulated time,
so the video plays at real speed whatever the load of the computer).

    record_gz_cameras.py --frames 1300 /viewcam=wide.mp4 /world/.../chase_cam=chase.mp4

Stops when every topic has the given number of frames. System python3 (Gazebo bindings).
"""
import argparse
import subprocess
import sys
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

ap = argparse.ArgumentParser()
ap.add_argument("--frames", type=int, required=True)
ap.add_argument("--timeout", type=float, default=900, help="real seconds to wait before giving up")
ap.add_argument("topics", nargs="+", help="topic=output.mp4")
a = ap.parse_args()

state = {}
node = Node()


def make_cb(topic, out):
    st = state[topic] = {"n": 0, "ff": None}

    def cb(m):
        if st["n"] >= a.frames:
            return
        if st["ff"] is None:
            st["ff"] = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (m.width, m.height),
                                         "-r", "20", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out], stdin=subprocess.PIPE)
        st["ff"].stdin.write(m.data)
        st["n"] += 1
    return cb


for item in a.topics:
    topic, out = item.split("=", 1)
    print(topic, node.subscribe(Image, topic, make_cb(topic, out)), flush=True)
t0 = time.time()
while any(s["n"] < a.frames for s in state.values()) and time.time() - t0 < a.timeout:
    time.sleep(0.2)
for s in state.values():
    if s["ff"]:
        s["ff"].stdin.close()
        s["ff"].wait()
print("frames", {k: v["n"] for k, v in state.items()})
sys.exit(0 if all(s["n"] >= a.frames for s in state.values()) else 1)
