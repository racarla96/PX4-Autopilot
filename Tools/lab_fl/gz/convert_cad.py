#!/usr/bin/env freecadcmd
"""Converts the CAD of the lab drone (ZMR250 frame, JMT 1806 motor, 4045 propeller, Pixhawk 2.4.8, printed legs) to the
STL meshes of the Gazebo model.

    CAD_DIR=~/Documents/GitHub/TFE_Mario_Bleda_Simulacion_Drone \
        freecadcmd -c "exec(open('Tools/lab_fl/gz/convert_cad.py').read())"

Output: Tools/lab_fl/gz/models/zmr250/meshes/*.stl, binary, in metres, in the body frame of the model (x forward, y left,
z up, origin at the centre of the frame, z = 0 on the top face of the arms). The meshes are committed, so this only has to
be run when a CAD changes.

Frame of the CAD of the frame: the long axis of the plates (Y of the CAD) is the flight direction, so
    body = (x: Y_cad, y: -X_cad, z: Z_cad).
Measured on the CAD: motor axes at (X, Y) = (+-98.8, +-78.3) mm, i.e. 252 mm between opposite motors.
"""
import os
import struct

import FreeCAD
import Import
import Mesh
import MeshPart

CAD_DIR = os.path.expanduser(os.environ.get("CAD_DIR", "~/Documents/GitHub/TFE_Mario_Bleda_Simulacion_Drone"))
OUT_DIR = os.environ.get("OUT_DIR", os.path.join(os.getcwd(), "Tools/lab_fl/gz/models/zmr250/meshes"))
DEFLECTION = float(os.environ.get("DEFLECTION", "0.4"))  # mm, tessellation tolerance

FRAME_STEP = "c_ZMR-250-Frame.STEP"
MOTOR_STEP = "c_JMT_Mini_Multi-Rotor_1806.STEP"
PROP_STEP = "c_Drone propeller.STEP"
PIXHAWK_STEP = "c_Pixhawk 2.4.8.STEP"
LEG_STL = "c_ZMR250_Landing_Gear.stl"

PROP_DIAMETER_MM = 101.6  # 4 inch; the CAD propeller is 317.5 mm long and is scaled to this
ARM_TOP_Z = 0.0           # top face of the arms in the CAD of the frame
BOTTOM_PLATE_BOTTOM_Z = -4.55


def write_stl(path, triangles, matrix, scale=0.001):
    """Binary STL; matrix: 3x3 rows (body_axis = sum matrix[i][j] * cad_j), applied after the translation of the caller."""
    with open(path, "wb") as f:
        f.write(b"zmr250 mesh, metres, body FLU".ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            p = [[scale * sum(matrix[i][j] * v[j] for j in range(3)) for i in range(3)] for v in tri]
            u = [p[1][k] - p[0][k] for k in range(3)]
            w = [p[2][k] - p[0][k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            ln = sum(c * c for c in n) ** 0.5 or 1.0
            f.write(struct.pack("<12fH", *[c / ln for c in n], *p[0], *p[1], *p[2], 0))
    print("wrote", path, len(triangles), "triangles")


# triangles kept per mesh; the CAD is far more detailed than a visual needs
TARGET_TRIANGLES = {"frame": 40000, "motor": 6000, "prop": 4000, "pixhawk": 6000, "leg_rear": 12000}


def decimate(tris, name):
    """Reduces the mesh to about TARGET_TRIANGLES[name] triangles (quadric edge collapse of FreeCAD's Mesh)."""
    if len(tris) <= TARGET_TRIANGLES[name]:
        return tris
    m = Mesh.Mesh([list(t) for t in tris])
    m.decimate(TARGET_TRIANGLES[name])
    return [tuple((p[0], p[1], p[2]) for p in f.Points) for f in m.Facets]


def triangles_of(shapes):
    out = []
    for s in shapes:
        verts, facets = s.tessellate(DEFLECTION)
        for a, b, c in facets:
            out.append((tuple(verts[a]), tuple(verts[b]), tuple(verts[c])))
    return out


def load_solids(step, exclude=()):
    doc = FreeCAD.newDocument()
    Import.insert(os.path.join(CAD_DIR, step), doc.Name)
    shapes = [o.Shape for o in doc.Objects if hasattr(o, "Shape") and o.Shape.Solids and o.Label not in exclude]
    return doc, shapes


def bbox_of(shapes):
    bb = FreeCAD.BoundBox()
    for s in shapes:
        bb.add(s.BoundBox)
    return bb


def shift(tris, d):
    return [tuple((v[0] + d[0], v[1] + d[1], v[2] + d[2]) for v in t) for t in tris]


def frame_properties(shapes, matrix):
    """Volume, centre of mass and inertia (unit density, about the centre of mass) of the frame from the exact CAD, in the
    body frame. make_model.py uses them: the tessellated and decimated STL does not keep the volume."""
    import json
    solids = [sol for sh in shapes for sol in sh.Solids]
    vol = sum(sol.Volume for sol in solids)
    com_cad = [sum(sol.Volume * sol.CenterOfMass[k] for sol in solids) / vol for k in range(3)]
    ic = [[0.0] * 3 for _ in range(3)]
    for sol in solids:  # parallel axis theorem, unit density, mm
        m_a = sol.MatrixOfInertia.A
        d = [sol.CenterOfMass[k] - com_cad[k] for k in range(3)]
        d2 = sum(x * x for x in d)
        for i in range(3):
            for j in range(3):
                ic[i][j] += m_a[4 * i + j] + sol.Volume * ((d2 if i == j else 0.0) - d[i] * d[j])
    r = [[matrix[i][j] for j in range(3)] for i in range(3)]
    ib = [[sum(r[i][a] * ic[a][b] * r[j][b] for a in range(3) for b in range(3)) / vol * 1e-6 for j in range(3)] for i in range(3)]
    com = [0.001 * sum(r[i][j] * com_cad[j] for j in range(3)) for i in range(3)]
    out = {"volume_m3": vol * 1e-9, "com_m": com, "inertia_per_mass_m2": ib}
    path = os.path.join(os.path.dirname(OUT_DIR.rstrip("/")), "..", "..", "data", "frame_cad_props.json")
    path = os.path.normpath(path)
    json.dump(out, open(path, "w"), indent=2)
    print("frame CAD volume cm3", vol * 1e-3, "com m", com, "->", path)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    body_from_frame_cad = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]

    # frame: plates, arms and spacers; 'Rahmen' is a merged copy of all of them and would double the mass
    doc, shapes = load_solids(FRAME_STEP, exclude=("Rahmen",))
    frame_properties(shapes, body_from_frame_cad)
    write_stl(os.path.join(OUT_DIR, "frame.stl"), decimate(triangles_of(shapes), "frame"), body_from_frame_cad)

    # motor: shaft axis along the CAD x axis; cylinder centred on the axis, base at z = 0, shaft up
    doc, shapes = load_solids(MOTOR_STEP)
    bb = bbox_of(shapes)
    yc, zc = bb.Center.y, bb.Center.z
    tris = shift(triangles_of(shapes), (-bb.XMin, -yc, -zc))  # axis x from 0..length
    # new axes: x_new = y_cad, y_new = z_cad, z_new = x_cad  (cyclic, right handed)
    write_stl(os.path.join(OUT_DIR, "motor.stl"), decimate(tris, "motor"), [[0, 1, 0], [0, 0, 1], [1, 0, 0]])
    print("motor length mm", bb.XLength, "diameter mm", max(bb.YLength, bb.ZLength))

    # propeller: blade along the CAD z axis, scaled from 317.5 mm to 4 inch, hub at the origin
    doc, shapes = load_solids(PROP_STEP)
    bb = bbox_of(shapes)
    c = bb.Center
    tris = shift(triangles_of(shapes), (-c.x, -c.y, -c.z))
    k = PROP_DIAMETER_MM / bb.ZLength
    # x_new = z_cad, y_new = x_cad, z_new = y_cad (cyclic)
    write_stl(os.path.join(OUT_DIR, "prop.stl"), decimate(tris, "prop"), [[0, 0, 1], [1, 0, 0], [0, 1, 0]], scale=0.001 * k)
    print("prop scale", k)

    # Pixhawk 2.4.8: the CAD stands the board up; lay it flat, centred
    doc, shapes = load_solids(PIXHAWK_STEP)
    bb = bbox_of(shapes)
    c = bb.Center
    tris = shift(triangles_of(shapes), (-c.x, -c.y, -c.z))
    write_stl(os.path.join(OUT_DIR, "pixhawk.stl"), decimate(tris, "pixhawk"), [[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    print("pixhawk bbox mm", bb.XLength, bb.YLength, bb.ZLength)

    # rear leg: the STL is one transverse bar of the landing gear at Y_cad ~ -80 (rear). z from -57.1 to 0 -> hangs from
    # the bottom plate. The front leg is this mesh rotated 180 degrees about z in the SDF.
    m = Mesh.Mesh(os.path.join(CAD_DIR, LEG_STL))
    tris = []
    for f in m.Facets:
        tris.append(tuple((p[0], p[1], p[2] + BOTTOM_PLATE_BOTTOM_Z) for p in f.Points))
    write_stl(os.path.join(OUT_DIR, "leg_rear.stl"), decimate(tris, "leg_rear"), body_from_frame_cad)


main()
