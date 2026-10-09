"""Simplify NVIDIA's official Jetson Nano Developer Kit (P3450, B01) STEP for the robot CAD.

Run (about 30 s):  python cad/simplify_jetson.py [--fan]
Input:  reference-models/jetson-nano-b01/Jetson_Nano_Dev_Kit_3D_b01.stp   (53.7 MB, 893 solids; unzip it from
        reference-models/Jetson_Nano_Dev_Kit_3D_b01.zip, it is not committed)
Output: reference-models/jetson-nano-b01/jetson-nano-b01-simplified.step  (one compound, solids in role order)
        reference-models/jetson-nano-b01/jetson-nano-b01-simplified.json  (roles -> solid indices + bboxes, mounting
        holes, heatsink fan-screw holes, port openings; build_robot.py reads both and checks the bboxes)

Frame (unchanged from NVIDIA's file, mm): carrier PCB x -4..96, y -17..62, z 0..1.57, component side +z; the I/O row
(barrel jack, HDMI/DP, 2 × stacked USB-A, RJ45, micro-USB) sits on the y = -17 long edge and faces -y.

Kept: every solid whose bounding box has a side >= 3 mm and is thicker than 0.2 mm, or whose volume is >= 5 mm3, plus
every named role below (carrier, module, heatsink, all connectors). Dropped: small SMD parts, solder pads and the
zero-thickness silkscreen/copper patches. --fan adds a plain 40 × 40 × 10 mm fan on the detected heatsink fan-screw
pattern (off by default: build_robot.py models the E09 fan itself from the same holes).
"""
from pathlib import Path
import json, sys, time
import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / 'reference-models' / 'jetson-nano-b01'
SRC = DIR / 'Jetson_Nano_Dev_Kit_3D_b01.stp'
OUT = DIR / 'jetson-nano-b01-simplified.step'
SIDE = DIR / 'jetson-nano-b01-simplified.json'
MIN_SIDE, MIN_THICK, MIN_VOL = 3.0, 0.2, 5.0
MAX_FACES = 300                       # solids with more B-rep faces become envelopes (connector pins and chamfers: the HDMI alone has 5289)
EXACT = {'carrier', 'module', 'heatsink', 'barrel_jack'}   # always kept face for face
IO = {'usb_a_0', 'usb_a_1', 'hdmi_dp', 'rj45', 'micro_usb'}   # envelopes keep their front openings as pockets
PCB_TOP = 1.57

# Named roles, found by bounding box in NVIDIA's frame (min x, y, z, max x, y, z; +-0.05 mm). The bboxes were read
# from the B01 file (zip SHA-256 73c30bf5...ac604); the script stops if a role is missing or ambiguous.
ROLES = {
    'carrier':      [(-4.0, -17.0, 0.0, 96.0, 62.0, 1.57)],
    'module':       [(9.2, 16.5, 6.87, 78.8, 61.5, 12.57)],
    'heatsink':     [(14.65, 21.6, 10.98, 73.35, 61.0, 28.28)],
    'sodimm':       [(5.25, 11.5, 1.2, 82.75, 37.35, 10.85)],
    'barrel_jack':  [(-2.5, -18.5, -1.93, 7.58, -3.87, 12.57)],
    'hdmi_dp':      [(10.8, -19.03, -0.9, 28.9, -0.47, 20.95)],
    'usb_a_0':      [(30.51, -18.5, -1.68, 46.69, -1.05, 17.94), (32.0, -18.25, 7.79, 45.2, -7.25, 12.07)],
    'usb_a_1':      [(47.51, -18.5, -1.68, 63.69, -1.05, 17.94), (49.0, -18.25, 7.79, 62.2, -7.25, 12.07)],
    'rj45':         [(64.85, -18.75, -2.23, 81.05, 2.85, 15.22)],
    'micro_usb':    [(84.0, -17.6, -0.1, 92.0, -10.7, 4.36)],
    'gpio_header':  [(83.46, 3.6, -1.43, 88.54, 54.4, 10.17)],
    'csi':          [(-2.3, 30.8, 1.65, 3.1, 53.2, 9.35), (-2.3, 6.8, 1.65, 3.1, 29.2, 9.35)],
    'fan_header':   [(63.12, 4.87, -1.93, 73.28, 10.67, 11.57)],
}


def bb(s):
    b = s.BoundingBox()
    return (b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax)


def r2(t):
    return [round(v, 2) for v in t]


def vertical_holes(s, rmin, rmax):
    """Centres (x, y), radius and z span of vertical cylindrical faces with rmin <= r <= rmax."""
    out = {}
    for f in s.Faces():
        if f.geomType() != 'CYLINDER':
            continue
        c = f._geomAdaptor().Cylinder()
        d, l = c.Axis().Direction(), c.Axis().Location()
        if abs(abs(d.Z()) - 1) < 1e-6 and rmin <= c.Radius() <= rmax:
            b = f.BoundingBox()
            out[(round(l.X(), 2), round(l.Y(), 2))] = dict(r=round(c.Radius(), 3), z=[round(b.zmin, 2), round(b.zmax, 2)])
    return out


def solid_runs(solids, x, y, z0, z1, step=0.05):
    """Solid intervals (z) along the vertical line at (x, y), sampled every step mm."""
    zs = [z0 + k * step for k in range(int((z1 - z0) / step) + 1)]
    inside = [any(s.isInside(cq.Vector(x, y, z), 1e-4) for s in solids) for z in zs]
    runs, start = [], None
    for k, z in enumerate(zs):
        if inside[k] and start is None:
            start = z
        if start is not None and (not inside[k] or k == len(zs) - 1):
            runs.append((start, zs[k - 1] if not inside[k] else z))
            start = None
    return runs


def solid_runs_x(s, x0, x1, y, z, step=0.05):
    """Solid intervals (x) along the horizontal line y, z."""
    f = classifier(s)
    xs = [x0 + k * step for k in range(int((x1 - x0) / step) + 1)]
    ins = [f(x, y, z) for x in xs]
    runs, start = [], None
    for k, x in enumerate(xs):
        if ins[k] and start is None:
            start = x
        if start is not None and (not ins[k] or k == len(xs) - 1):
            runs.append((start, xs[k - 1] if not ins[k] else x))
            start = None
    return runs


def usb_ports(runs):
    """USB-A port interiors from the shell section: thin runs (< 0.7 mm) are shell walls, a thick run between two walls
    is the contact tongue, so each port spans wall top .. next wall bottom around a tongue."""
    walls = [r for r in runs if r[1] - r[0] < 0.7]
    ports = []
    for a, b in zip(walls, walls[1:]):
        if any(r[1] - r[0] >= 1.5 and r[0] > a[1] and r[1] < b[0] for r in runs):
            ports.append((a[1], b[0]))
    return ports


def classifier(s):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    c = BRepClass3d_SolidClassifier(s.wrapped)
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON

    def inside(x, y, z):
        c.Perform(gp_Pnt(x, y, z), 1e-4)
        return c.State() in (TopAbs_IN, TopAbs_ON)
    return inside


def front_openings(solids, b, step=0.25, min_area=4.0):
    """Air regions enclosed by the connector body on the plane 1.5 mm behind its front face (y = b ymin + 1.5), as
    (x0, x1, z0, z1) rectangles: the sockets (HDMI, DP, USB-A, RJ45, micro-USB) seen from the front."""
    ins_f = [classifier(s) for s in solids]
    y, z0 = b[1] + 1.5, max(b[2], PCB_TOP)
    nx, nz = int((b[3] - b[0]) / step) + 1, int((b[5] - z0) / step) + 1
    xs = [b[0] + i * step for i in range(nx)]
    zs = [z0 + j * step for j in range(nz)]
    air = [[not any(f(x, y, z) for f in ins_f) for z in zs] for x in xs]
    seen = [[False] * nz for _ in range(nx)]

    def flood(i0, j0):
        st, cells = [(i0, j0)], []
        seen[i0][j0] = True
        while st:
            i, j = st.pop()
            cells.append((i, j))
            for a, c in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < nx and 0 <= c < nz and air[a][c] and not seen[a][c]:
                    seen[a][c] = True
                    st.append((a, c))
        return cells
    for i in range(nx):
        for j in range(nz):
            if (i in (0, nx - 1) or j in (0, nz - 1)) and air[i][j] and not seen[i][j]:
                flood(i, j)                          # air connected to the outside of the bbox is not a socket
    out = []
    for i in range(nx):
        for j in range(nz):
            if air[i][j] and not seen[i][j]:
                cells = flood(i, j)
                if len(cells) * step * step >= min_area:
                    out.append((xs[min(c[0] for c in cells)] - step / 2, xs[max(c[0] for c in cells)] + step / 2,
                                zs[min(c[1] for c in cells)] - step / 2, zs[max(c[1] for c in cells)] + step / 2))
    return sorted(out, key=lambda r: (r[2], r[0]))


def envelope(b, pockets=(), depth=0.0):
    """Bounding-box envelope of a connector or IC on the top side (clipped to the PCB top), minus front pockets."""
    z0 = max(b[2], PCB_TOP) if b[5] > PCB_TOP + 0.5 else b[2]
    env = cq.Workplane('XY').box(b[3] - b[0], b[4] - b[1], b[5] - z0, centered=False).translate((b[0], b[1], z0)).val()
    for x0, x1, q0, q1 in pockets:
        env = env.cut(cq.Workplane('XY').box(x1 - x0, depth + 0.1, q1 - q0, centered=False).translate((x0, b[1] - 0.1, q0)).val())
    return env


def main():
    add_fan = '--fan' in sys.argv
    t0 = time.time()
    if not SRC.exists():
        sys.exit(f'{SRC} missing: unzip reference-models/Jetson_Nano_Dev_Kit_3D_b01.zip into {DIR}')
    sol = cq.importers.importStep(str(SRC)).solids().vals()
    print(f'loaded {len(sol)} solids in {time.time() - t0:.1f} s')
    boxes = [bb(s) for s in sol]

    role_idx = {}
    for role, targets in ROLES.items():
        idx = []
        for t in targets:
            hit = [i for i, b in enumerate(boxes) if all(abs(b[k] - t[k]) <= 0.05 for k in range(6))]
            if len(hit) != 1:
                sys.exit(f'role {role}: {len(hit)} solids match bbox {t}')
            idx += hit
        role_idx[role] = idx
    named = {i for v in role_idx.values() for i in v}

    def keep(i):
        b = boxes[i]
        d = sorted((b[3] - b[0], b[4] - b[1], b[5] - b[2]))
        return i in named or (d[2] >= MIN_SIDE and d[0] >= MIN_THICK) or sol[i].Volume() >= MIN_VOL
    other = [i for i in range(len(sol)) if i not in named and keep(i)]
    order = [i for v in role_idx.values() for i in v] + other
    print(f'kept {len(order)} of {len(sol)} solids ({len(named)} named, {len(other)} other >= {MIN_SIDE} mm side '
          f'and >= {MIN_THICK} mm thick, or >= {MIN_VOL} mm3); dropped {len(sol) - len(order)}')

    # Features in NVIDIA's frame.
    carrier, sink = sol[role_idx['carrier'][0]], sol[role_idx['heatsink'][0]]
    holes = vertical_holes(carrier, 1.3, 1.45)
    mount = sorted(p for p in holes if any(abs(p[0] - a) < .05 and abs(p[1] - b) < .05 for a, b in [(0, 0), (86, 0), (0, 58), (86, 58)]))
    assert len(mount) == 4, mount
    top = bb(sink)[5]
    fan_holes = sorted(p for p, h in vertical_holes(sink, 1.2, 1.6).items() if h['z'][1] > top - 2)
    assert len(fan_holes) == 4, fan_holes
    fan_c = (sum(p[0] for p in fan_holes) / 4, sum(p[1] for p in fan_holes) / 4)
    fan_pitch = (max(p[0] for p in fan_holes) - min(p[0] for p in fan_holes), max(p[1] for p in fan_holes) - min(p[1] for p in fan_holes))
    ports, sockets = {}, {}
    for role in ('usb_a_0', 'usb_a_1'):
        b = boxes[role_idx[role][0]]
        cx = (b[0] + b[3]) / 2
        pz = usb_ports(solid_runs([sol[i] for i in role_idx[role]], cx, b[1] + 3.5, b[2], b[5]))
        assert len(pz) == 2, pz   # stacked: lower and upper port
        ports[role] = dict(face_y=round(b[1], 2), x=round(cx, 2), ports_z=[r2(z) for z in pz], centres_z=[round((z[0] + z[1]) / 2, 2) for z in pz])
        sockets[role] = [(cx - 6.25, cx + 6.25, z[0], z[1]) for z in pz]   # USB-A receptacle opening 12.5 mm wide (USB 2.0 spec)
    # micro-USB B: walls from a vertical and a horizontal probe 1 mm behind the face
    mb = boxes[role_idx['micro_usb'][0]]
    mu = sol[role_idx['micro_usb'][0]]
    zr = solid_runs([mu], (mb[0] + mb[3]) / 2, mb[1] + 1.5, PCB_TOP, mb[5])
    zc = (zr[0][1] + zr[-1][0]) / 2
    xr = solid_runs_x(mu, mb[0], mb[3], mb[1] + 1.0, zc)
    sockets['micro_usb'] = [(xr[0][1], xr[-1][0], zr[0][1], zr[-1][0])]
    jack = sol[role_idx['barrel_jack'][0]]
    pin = [f for f in jack.Faces() if f.geomType() == 'CYLINDER' and abs(abs(f._geomAdaptor().Cylinder().Axis().Direction().Y()) - 1) < 1e-6
           and 3.0 < f._geomAdaptor().Cylinder().Radius() < 3.4]
    ax = pin[0]._geomAdaptor().Cylinder().Axis().Location()
    ports['barrel_jack'] = dict(face_y=round(boxes[role_idx['barrel_jack'][0]][1], 2), x=round(ax.X(), 2), z=round(ax.Z(), 2),
                                bore_d=round(2 * pin[0]._geomAdaptor().Cylinder().Radius(), 2))
    fh = boxes[role_idx['fan_header'][0]]
    base = solid_runs([sol[role_idx['fan_header'][0]]], (fh[0] + fh[3]) / 2, (fh[1] + fh[4]) / 2, PCB_TOP - 0.5, fh[5])
    ports['fan_header'] = dict(centre_xy=r2(((fh[0] + fh[3]) / 2, (fh[1] + fh[4]) / 2)), size=r2((fh[3] - fh[0], fh[4] - fh[1])), top_z=round(fh[5], 2),
                               base_top_z=round(base[0][1], 2))   # plastic base; the mated plug sits on it over the pins

    # Heavy solids (> MAX_FACES faces) outside EXACT become bounding-box envelopes; I/O connectors keep their sockets as
    # pockets (probed on the exact solids). A role made of several solids (USB-A shell + inner block) becomes one envelope.
    out, roles, src, simplified = [], {}, [], {}
    for role, v in role_idx.items():
        heavy = role not in EXACT and any(len(sol[i].Faces()) > MAX_FACES for i in v)
        roles[role] = []
        if heavy and role in IO:
            b = boxes[v[0]]
            pk = sockets[role] if role in sockets else front_openings([sol[i] for i in v], b)
            depth = min(12.0, 0.6 * (b[4] - b[1]))
            roles[role].append(len(out)); out.append(envelope(b, pk, depth)); src.append(v)
            simplified[role] = dict(kind='envelope with front sockets', faces_before=sum(len(sol[i].Faces()) for i in v), sockets_xz=[r2(p) for p in pk], socket_depth=round(depth, 2))
        elif heavy:
            for i in v:
                roles[role].append(len(out)); out.append(envelope(boxes[i])); src.append([i])
            simplified[role] = dict(kind='bounding-box envelope', faces_before=sum(len(sol[i].Faces()) for i in v))
        else:
            for i in v:
                roles[role].append(len(out)); out.append(sol[i]); src.append([i])
    roles['other'] = []
    for i in other:
        roles['other'].append(len(out)); src.append([i])
        if len(sol[i].Faces()) > MAX_FACES:
            out.append(envelope(boxes[i])); simplified[f'other_{i}'] = dict(kind='bounding-box envelope', faces_before=len(sol[i].Faces()))
        else:
            out.append(sol[i])
    print(f'{len(simplified)} heavy solids replaced by envelopes; faces {sum(len(sol[i].Faces()) for i in order)} -> {sum(len(s.Faces()) for s in out)}')
    for r, d in simplified.items():
        if 'sockets_xz' in d:
            print(f'  {r}: sockets (x0, x1, z0, z1) {d["sockets_xz"]}, {d["socket_depth"]} deep')
    if add_fan:
        z = top
        fan = cq.Workplane('XY').box(40, 40, 10, centered=(True, True, False)).translate((*fan_c, z)).val()
        fan = fan.cut(cq.Solid.makeCylinder(19, 12, cq.Vector(*fan_c, z - 1)))
        for x, y in fan_holes:
            fan = fan.cut(cq.Solid.makeCylinder(1.7, 12, cq.Vector(x, y, z - 1)))
        fan = fan.fuse(cq.Solid.makeCylinder(7, 10, cq.Vector(*fan_c, z)))
        roles['fan'] = [len(out)]
        out.append(fan)
    comp = cq.Compound.makeCompound(out)
    comp.exportStep(str(OUT), write_pcurves=False)   # no p-curves: about a third of the size; OCC rebuilds them on import

    # Re-import and confirm the solid order survives the STEP round trip.
    back = cq.importers.importStep(str(OUT)).solids().vals()
    assert len(back) == len(out), (len(back), len(out))
    for a, b in zip(back, out):
        assert all(abs(p - q) < 0.02 for p, q in zip(bb(a), bb(b))), (bb(a), bb(b))
    side = dict(source=SRC.name, source_solids=len(sol), solids=len(out),
                rule=f'named roles + bbox side >= {MIN_SIDE} mm and thickness >= {MIN_THICK} mm, or volume >= {MIN_VOL} mm3; solids with > {MAX_FACES} faces (except {sorted(EXACT)}) as envelopes',
                frame='NVIDIA P3450 B01 STEP frame, mm: carrier PCB x -4..96, y -17..62, z 0..1.57 (component side +z); I/O row on the y = -17 edge, facing -y',
                roles=roles, bboxes=[r2(bb(s)) for s in out], source_index=src + ([None] if add_fan else []), envelopes=simplified,
                max_faces=MAX_FACES, exact_roles=sorted(EXACT),
                mounting_holes=dict(xy=[list(p) for p in mount], d=round(2 * holes[mount[0]]['r'], 2)),
                module_holes=[list(p) for p in holes if p not in mount],
                heatsink=dict(top_z=round(top, 2), fan_screw_holes=[list(p) for p in fan_holes], fan_centre=r2(fan_c), fan_pitch=r2(fan_pitch),
                              fan_hole_d=round(2 * vertical_holes(sink, 1.2, 1.6)[fan_holes[0]]['r'], 2)),
                ports=ports, fan_included=add_fan)
    SIDE.write_text(json.dumps(side, indent=1))
    print('roles:', {r: len(v) for r, v in roles.items()})
    for r, v in roles.items():
        if r != 'other':
            print(f'  {r:12s}', r2(bb(out[v[0]])))
    print('mounting holes', side['mounting_holes'], '\nfan screws', side['heatsink'])
    print('ports', json.dumps(ports))
    total = bb(comp)
    print('simplified bbox', r2(total), f'-> {OUT.relative_to(ROOT)} ({OUT.stat().st_size / 1e6:.1f} MB), {time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
