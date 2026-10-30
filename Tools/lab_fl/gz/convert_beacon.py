#!/usr/bin/env freecadcmd
"""Converts the Marvelmind Super-Beacon STL (from marvelmind_products_PDF_STEP_STL_PNG.zip) to the mesh of the Gazebo model.

    MARVELMIND_DIR=/path/to/products_PDF_STEP_STL_PNG \
        freecadcmd -c "exec(open('Tools/lab_fl/gz/convert_beacon.py').read())"

Output: models/zmr250/meshes/beacon.stl, binary, metres, ~6000 triangles, centred on x and y with the bottom face at z = 0
(the antenna points up). The housing is 55 x 55 x 32 mm; with the antenna the mesh is 100 mm tall.
"""
import os
import struct

import Mesh

SRC = os.path.expanduser(os.environ["MARVELMIND_DIR"]) + "/STL/Super-Beacon_body.STL"
OUT = os.environ.get("OUT", os.path.join(os.getcwd(), "Tools/lab_fl/gz/models/zmr250/meshes/beacon.stl"))
TARGET = 6000

m = Mesh.Mesh(SRC)
m.decimate(TARGET)
bb = m.BoundBox
cx, cy, z0 = (bb.XMin + bb.XMax) / 2, (bb.YMin + bb.YMax) / 2, bb.ZMin
tris = [[(p[0] - cx, p[1] - cy, p[2] - z0) for p in f.Points] for f in m.Facets]
with open(OUT, "wb") as f:
    f.write(b"Marvelmind Super-Beacon, metres".ljust(80, b" "))
    f.write(struct.pack("<I", len(tris)))
    for t in tris:
        p = [[0.001 * c for c in v] for v in t]
        u = [p[1][k] - p[0][k] for k in range(3)]
        w = [p[2][k] - p[0][k] for k in range(3)]
        n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        ln = sum(c * c for c in n) ** 0.5 or 1.0
        f.write(struct.pack("<12fH", *[c / ln for c in n], *p[0], *p[1], *p[2], 0))
print("wrote", OUT, len(tris), "triangles; size mm", bb.XLength, bb.YLength, bb.ZLength)
