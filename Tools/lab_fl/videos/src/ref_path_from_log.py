#!/usr/bin/env python3
"""Reference path of a flight log as a JSON list of NED points, one every 0.5 s while the trajectory mode is active.

    ref_path_from_log.py flight.ulg ../data/ref_paper2.json

Used by spawn_gz_scene.py to draw the reference as dots in Gazebo. Needs pyulog and numpy.
"""
import json
import sys

import numpy as np
from pyulog import ULog

log = ULog(sys.argv[1], message_name_filter_list=["traj_control_status"])
d = log.data_list[0].data
act = d["active"] > 0
t = d["timestamp"][act] / 1e6
t -= t[0]
ref = np.column_stack([d["position_ref[%d]" % i][act] for i in range(3)])
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 35.5
idx = [int(np.argmin(np.abs(t - k * 0.5))) for k in range(int(min(t[-1], dur) * 2) + 1)]
json.dump(ref[idx].round(3).tolist(), open(sys.argv[2], "w"))
print(len(idx), "points")
