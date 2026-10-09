"""Board outline = envelope rectangle minus the real footprint of every remaining CAD part that reaches into the board slab.

Footprint of a part = 2D convex hull of its mesh cross-sections through the slab (z levels every <= 2 mm, plus the slab boundaries, the height
zone ceilings and every mesh vertex inside the slab, so thin flanges cannot fall between levels), taken from cad/assembly-meshes.json (the real
geometry, not bounding boxes). The slab runs from the top of P_base (z 33, under the board's 2.4 mm spacers) to the Jetson carrier's lowest lead
(B.UNDER_CARRIER_Z, z 54.77). Inside a board_spec.HEIGHT_ZONES box the slab is only as tall as that zone allows (a zone that reaches a board edge
continues past it), so what hangs at or above the zone ceiling is a ceiling there, not a keep-out.
The envelope is robot x -70..7.5, y -50..50: the nominal 74 x 100 mm frame, widened to x -70 at the rear. The meshes cut the rear (the battery tray's
front wall, x -68: the edge lands at x -66.4) and the two notches beside the tongue (the motors, |y| >= 35: |y| <= 33.4); the 100 mm sides are the
envelope itself. The front edge IS the frame's x 7.5: the P03 regulator (x 9) sits exactly MARGIN off it, so it is not a keep-out hull (a
1.5 mm pre-filter keeps a part that only touches the margin out) and the real-geometry test checks the 1.5 mm. (A front envelope at x 10 lets the
opening's disks bulge to x 7.73 past the frame at the tongue corners, over the 74 mm limit.)
The placeholders the real board replaces (board_spec.REMOVED_CAD_IDS), the harness wires and the parts in NOT_OBSTACLE_IDS (the clamp stack, the
bearing ribs, the inserts, the plugged motor cables, the mated plugs and the rear-edge hold-down: reasons at the lists) are skipped; everything else that stays is a keep-out.
Parts of the floating bumper (ids 'bumper_*' / 'P_bumper*') are swept along their 5 mm travel first. Each hull is inflated by 1.5 mm and cut from
the envelope rectangle with CadQuery 2D booleans: the outer wire is the outline, inner wires (keep-outs wholly inside the board) are cutouts.
Parts whose xy bounds exceed COMPACT are shells (bumper skins, lid sectors ...) and must not reach the envelope at all, except the explicit
WINDOWED list (motors, P03, battery tray, battery and straps): for those only the material inside a 5 mm window around the envelope is hulled.
Fabricability (ruling R10): gaps narrower than 2 * CLOSE between keep-outs, or between a keep-out and the envelope edge, are closed before the
outline is taken: the board is replaced by its morphological opening with a disk of radius CLOSE (erode by CLOSE, dilate by CLOSE), which is the
same as cutting the closing of (keep-outs + outside of the envelope). An opening only ever removes board material, so no keep-out shrinks; it
deletes teeth and necks narrower than 2 * CLOSE and rounds convex board corners to radius CLOSE. The wires are then sampled so that no edge is
shorter than MIN_EDGE (sharp corners kept, curves resampled with chords that stay within CHORD of the true curve, which the keep-out inflation
covers).
Holes: the four M3 holes (H1..H4) sit on the heat-set inserts insert_bay_0..3 in P_base and must not move; the two clamp holes (H5, H6; the
board is clamped between a printed spacer and the rear Jetson standoff, M2.5 screw, drilled O3.0 NPTH in B3) sit on the centres of the rear
standoffs jetson_standoff_3 / jetson_standoff_1. Both kinds must keep the 4 mm edge rule or the run stops.
Run with the CAD venv:  $CADPY pcb/extract_outline.py   (writes pcb/board_outline.json)."""
import json, math, sys
from pathlib import Path
import numpy as np
import trimesh
import cadquery as cq
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pcb import board_spec as B

MARGIN = 1.5                                    # keep-out inflation (mm)
SIMPLIFY = 0.05                                 # hull simplification tolerance (mm); the cut is inflated by MARGIN + SIMPLIFY + CHORD so the 1.5 mm margin to the true hull is kept
CHORD = 0.05                                    # max distance between the written polygon and the true (arc) boundary (mm)
CLOSE = 1.1                                     # opening radius (mm): no board neck / tooth narrower than 2 * CLOSE = 2.2 mm survives (R10 asks for 2 mm)
MIN_EDGE = 0.3                                  # shortest outline edge written (mm)
DZ = 2.0                                        # section spacing (mm)
COMPACT = 40.0                                  # a part whose xy bounds exceed this (mm) is a shell: it must not reach the envelope at all (unless it is in WINDOWED)
Z0, Z1 = 33.0, B.UNDER_CARRIER_Z               # board + components slab (robot z): from the P_base top (the ribs, spacers and the tray wall stand on it) to the carrier's lowest lead
ENV = (-70.0, B.BOARD_Y0 - B.BOARD_H, B.BOARD_X0 + B.BOARD_W, B.BOARD_Y0)   # robot-frame xmin, ymin, xmax, ymax: x -70..7.5, y -50..50
assert ENV[0] <= B.BOARD_X0 and ENV[2] >= B.BOARD_X0 + B.BOARD_W and (ENV[1], ENV[3]) == (B.BOARD_Y0 - B.BOARD_H, B.BOARD_Y0), 'the envelope must contain the nominal board'
PAR = json.loads((ROOT / 'cad/parameters.json').read_text())
MAN = {c['id']: c['bounds'] for c in json.loads((ROOT / 'cad/component-manifest.json').read_text())}
TRAVEL, AXIS = PAR['bumper_travel'], math.radians(PAR['bumper_axis_deg'])   # build_robot.py: ft=(-cos BA, -sin BA) left half; right half mirrors y
SWEEP_S = np.linspace(0.0, 1.0, 11)
WINDOW = 5.0                                    # mm around the envelope: material farther out can neither bite the board (1.5) nor change the opening (2 * CLOSE)
WINDOWED = {'P_battery_tray', 'battery', 'strap_0', 'strap_1', 'motor_left', 'motor_right', 'p03_regulator'}   # big parts that are hulled inside the window only (their hull over the whole part is meaningless); the battery's window hull is a bare line (x -71), too thin to list, 4.6 mm off the board
def centre(i): b = MAN[i]; return ((b[0] + b[3]) / 2, (b[1] + b[4]) / 2)
HOLE_SPOTS_ROBOT = [centre(f'insert_bay_{i}') for i in range(4)]       # the heat-set inserts in P_base (= BAY_MOUNTS), order H1..H4
CLAMP_SPOTS_ROBOT = [centre('jetson_standoff_3'), centre('jetson_standoff_1')]   # the two rear Jetson standoffs (= BAY_CLAMPS), order H5 (left, y +40), H6 (right, y -46)
HOLE_EDGE = 4.0                                 # mm from every outline / cutout edge (R17 hole rule)
# Parts that stay in the CAD and sit in the slab over or under the board but are NOT obstacles to its outline; each is left out of the keep-out hulls:
CLAMP_STACK_IDS = ['jetson_standoff_1', 'jetson_standoff_3', 'jetson_screw_1', 'jetson_screw_3']
#   the rear Jetson standoff stacks stand ON the board and their M2.5 screws go through it: they ARE the clamp (-> clamp_holes_uv), not a cutout.
#   (The printed clamp spacers bay_clamp_spacer_1/3 under the board are in B.REMOVED_CAD_IDS.)
SUPPORT_IDS = ['bay_rib_0', 'bay_rib_1'] + [f'insert_bay_{i}' for i in range(4)]
#   the printed bearing ribs (z 33..35.4) carry the board under its rear-edge connector row by design, and the heat-set inserts (z 30..33) sit in P_base
#   below the board: both are under it, and the board rests on them.
PLUGGED_CABLE_IDS = ['motor_cable_left', 'motor_cable_right', 'motor_pigtail_left', 'motor_pigtail_right']
#   the factory motor cables and (PB25) the motor pigtails plugged into J4 / J5 run over the board's motor-notch edges (z 37..53); unlike the harness
#   wires they are not W-prefixed.
BOARD_CONTACT_IDS = ['bay_holddown'] + [i for i in MAN if i.startswith('bay_plug_')]
#   bay B8: the mated plugs on the board's own connectors (bay_plug_<ref>) stand on the board, and the battery tray's hold-down fingers
#   (bay_holddown) press its rear edge down onto the ribs: designed contacts with the board, like the ribs.
NOT_OBSTACLE_IDS = set(CLAMP_STACK_IDS) | set(SUPPORT_IDS) | set(PLUGGED_CABLE_IDS) | set(BOARD_CONTACT_IDS)
skip = set(B.REMOVED_CAD_IDS) | {'P_base'} | NOT_OBSTACLE_IDS
BIG = 1e6
def zone_rect(z):
    u0, v0, u1, v1, h = z
    x0, x1 = (-BIG if u0 <= 0 else B.BOARD_X0 + u0), (BIG if u1 >= B.BOARD_W else B.BOARD_X0 + u1)
    y0, y1 = (-BIG if v1 >= B.BOARD_H else B.BOARD_Y0 - v1), (BIG if v0 <= 0 else B.BOARD_Y0 - v0)
    e = 0.01      # material exactly on a zone edge counts as inside the zone
    return (x0 - e, y0 - e, x1 + e, y1 + e), B.Z_PCB_TOP + h
ZONES = [zone_rect(z) for z in B.HEIGHT_ZONES]       # robot-frame rectangle and the z above which that rectangle is not part of the slab

def hull(pts):
    """Convex hull (monotone chain), counter-clockwise, as a list of (x, y)."""
    P = sorted({(float(x), float(y)) for x, y in np.round(np.asarray(pts, float), 6)})
    if len(P) < 3: return P
    def cross(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

def simplify_ring(poly, eps):
    """Douglas-Peucker on a closed polygon: result deviates from the input by <= eps (Hausdorff), so inflating it by MARGIN + eps still covers the input inflated by MARGIN."""
    def rdp(pts):
        (ax, ay), (bx, by) = pts[0], pts[-1]
        dx, dy, L = bx - ax, by - ay, math.hypot(bx - ax, by - ay)
        dmax, k = 0.0, 0
        for j in range(1, len(pts) - 1):
            d = abs(dx * (pts[j][1] - ay) - dy * (pts[j][0] - ax)) / L if L else math.dist(pts[j], pts[0])
            if d > dmax: dmax, k = d, j
        return rdp(pts[:k + 1])[:-1] + rdp(pts[k:]) if dmax > eps else [pts[0], pts[-1]]
    if len(poly) < 4: return poly
    far = max(range(len(poly)), key=lambda j: math.dist(poly[j], poly[0]))
    return rdp(poly[:far + 1])[:-1] + rdp(poly[far:] + poly[:1])[:-1]

def clip_to_rect(poly, rect):
    """Sutherland-Hodgman: convex polygon clipped by an axis-aligned rectangle (xmin, ymin, xmax, ymax)."""
    x0, y0, x1, y1 = rect
    for axis, lim, keep_ge in ((0, x0, True), (0, x1, False), (1, y0, True), (1, y1, False)):
        out = []
        for a, b in zip(poly, poly[1:] + poly[:1]):
            ina, inb = (a[axis] >= lim) == keep_ge, (b[axis] >= lim) == keep_ge
            if ina != inb:
                t = (lim - a[axis]) / (b[axis] - a[axis])
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            if inb: out.append(b)
        poly = out
        if not poly: return []
    return poly

def seg_hits_rect(a, b, rect):
    """Liang-Barsky: does segment a-b touch the rectangle?"""
    t0, t1, d = 0.0, 1.0, (b[0] - a[0], b[1] - a[1])
    for p, q in ((-d[0], a[0] - rect[0]), (d[0], rect[2] - a[0]), (-d[1], a[1] - rect[1]), (d[1], rect[3] - a[1])):
        if abs(p) < 1e-12:
            if q < 0: return False
        else:
            t = q / p
            if p < 0: t0 = max(t0, t)
            else: t1 = min(t1, t)
            if t0 > t1: return False
    return True

def area_signed(poly): return sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1])) / 2
def inside(poly, p):
    x, y = p; c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1: c = not c
    return c
def dist_to_boundary(poly, p):
    best = 1e9
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - x1 - t * dx, p[1] - y1 - t * dy))
    return best

def clip_inside(S, rect):
    """Segments (n,2,2) clipped to the closed rectangle (Liang-Barsky, vectorised): (m,2,2) of the pieces inside."""
    a, d = S[:, 0], S[:, 1] - S[:, 0]
    t0, t1, ok = np.zeros(len(S)), np.ones(len(S)), np.ones(len(S), bool)
    for k, (lo, hi) in enumerate(((rect[0], rect[2]), (rect[1], rect[3]))):
        par = np.abs(d[:, k]) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            ta, tb = (lo - a[:, k]) / d[:, k], (hi - a[:, k]) / d[:, k]
        t0 = np.maximum(t0, np.where(par, -np.inf, np.minimum(ta, tb))); t1 = np.minimum(t1, np.where(par, np.inf, np.maximum(ta, tb)))
        ok &= ~(par & ((a[:, k] < lo - 1e-9) | (a[:, k] > hi + 1e-9)))
    ok &= t1 >= t0 - 1e-12
    p0, p1 = a + t0[:, None] * d, a + t1[:, None] * d
    return np.stack([p0, p1], 1)[ok]

def clip_outside(S, rect):
    """Segments (n,2,2): the pieces outside the OPEN rectangle (a piece on its boundary is kept), as (m,2,2)."""
    a, d = S[:, 0], S[:, 1] - S[:, 0]
    t0, t1, ok = np.zeros(len(S)), np.ones(len(S)), np.ones(len(S), bool)
    for k, (lo, hi) in enumerate(((rect[0], rect[2]), (rect[1], rect[3]))):
        par = np.abs(d[:, k]) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            ta, tb = (lo - a[:, k]) / d[:, k], (hi - a[:, k]) / d[:, k]
        t0 = np.maximum(t0, np.where(par, -np.inf, np.minimum(ta, tb))); t1 = np.minimum(t1, np.where(par, np.inf, np.maximum(ta, tb)))
        ok &= ~(par & ((a[:, k] <= lo + 1e-9) | (a[:, k] >= hi - 1e-9)))     # parallel to a face and on or outside it: never enters the open rectangle
    cut = ok & (t1 > t0 + 1e-9)                                               # the segment passes through the open rectangle on [t0, t1]
    keep = [S[~cut]]
    head = cut & (t0 > 1e-9); tail = cut & (t1 < 1 - 1e-9)
    keep.append(np.stack([a[head], a[head] + t0[head, None] * d[head]], 1))
    keep.append(np.stack([a[tail] + t1[tail, None] * d[tail], S[tail, 1]], 1))
    return np.concatenate(keep)

def in_open_rect(P, rect):
    return (P[:, 0] > rect[0]) & (P[:, 0] < rect[2]) & (P[:, 1] > rect[1]) & (P[:, 1] < rect[3])

def slab_geometry(m):
    """(section segments (n,2,2), points (k,2)) of a part's mesh inside the slab, robot frame, at rest. Over a height zone only the mesh below
    that zone's ceiling counts (a segment at height z loses the stretch that lies inside a zone whose ceiling is below z)."""
    V = np.asarray(m['vertices'], float).reshape(-1, 3)
    mesh = trimesh.Trimesh(V, np.asarray(m['faces'], int).reshape(-1, 3), process=False)
    zlo, zhi = max(Z0, m['bounds'][2]), min(Z1, m['bounds'][5])
    extra = [Z0, Z1] + [zc for _, zc in ZONES if Z0 < zc < Z1]
    levels = sorted({round(z, 6) for z in np.append(np.arange(Z0, Z1, DZ), extra) if zlo - 1e-9 <= z <= zhi + 1e-9})
    parts = []
    for z in levels:
        seg = trimesh.intersections.mesh_plane(mesh, [0, 0, 1], [0, 0, z])
        if not len(seg): continue
        seg = seg[:, :, :2]
        for rect, zc in ZONES:
            if z > zc + 1e-9: seg = clip_outside(seg, rect)
        parts.append(seg)
    segs = np.concatenate(parts) if parts else np.zeros((0, 2, 2))
    keepv = (V[:, 2] >= Z0 - 1e-3) & (V[:, 2] <= Z1 + 1e-3)     # a flange top exactly 1 mm under the board (z = Z0) counts
    for rect, zc in ZONES: keepv &= ~((V[:, 2] > zc + 1e-9) & in_open_rect(V[:, :2], rect))
    ins = V[keepv][:, :2]
    pts = np.concatenate([segs.reshape(-1, 2), ins]) if len(segs) else ins
    return segs, pts

def travel_dir(m):
    """Unit travel direction of a floating-bumper part: left half (y > 0) (-cos a, -sin a), right half (y < 0) (-cos a, +sin a)."""
    left = (m['bounds'][1] + m['bounds'][4]) / 2 > 0
    assert left == ('left' in m['id']) and left != ('right' in m['id']), (m['id'], 'left/right mismatch')
    return np.array([-math.cos(AXIS), -math.sin(AXIS) if left else math.sin(AXIS)])

def swept(a, d):
    """Copies of array a (..., 2) shifted by s*TRAVEL*d for s in SWEEP_S, stacked along a new leading axis."""
    return np.stack([a + s * TRAVEL * d for s in SWEEP_S])

meshes = json.loads((ROOT / 'cad/assembly-meshes.json').read_text())
infl = (ENV[0] - MARGIN, ENV[1] - MARGIN, ENV[2] + MARGIN, ENV[3] + MARGIN)     # envelope grown by the margin: a hull this close still bites the board
win = (ENV[0] - WINDOW, ENV[1] - WINDOW, ENV[2] + WINDOW, ENV[3] + WINDOW)
keep, shells = [], []
for m in meshes:
    i, (x0, y0, z0, x1, y1, z1) = m['id'], m['bounds']
    if i in skip or i.startswith('W') or z1 < Z0 or z0 > Z1: continue
    moves = i.startswith('bumper_') or i.startswith('P_bumper')
    d = travel_dir(m) if moves else np.zeros(2)
    reach = TRAVEL if moves else 0.0
    big = max(x1 - x0, y1 - y0) > COMPACT
    box = win if i in WINDOWED else infl
    if x1 + reach < box[0] or x0 - reach > box[2] or y1 + reach < box[1] or y0 - reach > box[3]: continue
    segs, pts = slab_geometry(m)
    if not len(pts): continue
    if big and i in WINDOWED:                   # hull of the material inside the window only
        S = clip_inside(swept(segs, d).reshape(-1, 2, 2), win).reshape(-1, 2) if len(segs) else np.zeros((0, 2))
        P = swept(pts, d).reshape(-1, 2)
        P = np.concatenate([S, P[(P[:, 0] >= win[0]) & (P[:, 0] <= win[2]) & (P[:, 1] >= win[1]) & (P[:, 1] <= win[3])]])
        if not len(P): shells.append(i); continue
        H = hull(P)
    elif big:                                   # shell (bumper skins, lid sectors, foam): only the section segments matter, and none may touch the envelope
        S = swept(segs, d).reshape(-1, 2, 2)
        near = S[(S[:, :, 0].max(1) >= infl[0]) & (S[:, :, 0].min(1) <= infl[2]) & (S[:, :, 1].max(1) >= infl[1]) & (S[:, :, 1].min(1) <= infl[3])]   # cheap bbox prefilter
        if any(seg_hits_rect(a, b, infl) for a, b in near): raise RuntimeError(f'{i}: non-compact part reaches the board envelope; handle it explicitly (WINDOWED)')
        shells.append(i); continue
    else:
        H = hull(swept(pts, d).reshape(-1, 2))
    if len(H) < 3 or not clip_to_rect(H, infl): continue
    keep.append(dict(id=i, bounds=[round(v, 3) for v in m['bounds']], hull_xy=[[round(x, 3), round(y, 3)] for x, y in H]))

def poly_face(pts): return cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(x, y, 0) for x, y in pts], close=True))
def inflated_face(pts):
    pts = simplify_ring(list(pts), SIMPLIFY)
    w = cq.Wire.makePolygon([cq.Vector(x, y, 0) for x, y in pts], close=True).offset2D(MARGIN + SIMPLIFY + CHORD, 'arc')[0]
    f = cq.Face.makeFromWires(w)
    assert f.Area() > poly_face(pts).Area(), 'offset went the wrong way'
    return f

def wire_chain(wire):
    """Dense closed polyline (robot x, y) of a planar wire: line edges by their end points, curved edges sampled every <= 0.05 mm
    (sagitta < 0.0003 mm at the 1.1 mm smallest radius). Edges are chained by endpoint matching."""
    chains = []
    for e in wire.Edges():
        n = 1 if e.geomType() == 'LINE' else max(2, math.ceil(e.Length() / 0.05))
        c = [e.positionAt(k / n) for k in range(n + 1)]      # positionAt follows the edge orientation (startPoint/endPoint of an arc may not)
        chains.append([(p.x, p.y) for p in c])
    out = chains.pop(0)
    while chains:
        for k, c in enumerate(chains):
            if math.dist(c[0], out[-1]) < 1e-4: out += c[1:]; break
            if math.dist(c[-1], out[-1]) < 1e-4: out += c[::-1][1:]; break
        else: raise RuntimeError('wire edges do not chain')
        chains.pop(k)
    assert math.dist(out[0], out[-1]) < 1e-4
    return [p for i, p in enumerate(out[:-1]) if i == 0 or math.dist(p, out[i - 1]) > 1e-6]

def turn_deg(a, b, c):
    """Turning angle (deg) of the polyline a-b-c at b."""
    return math.degrees(abs(math.atan2((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]),
                                       (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]))))

def wire_polygon(wire):
    """Closed polygon of a wire with no edge shorter than MIN_EDGE: sharp corners (> 20 deg turn in the dense chain) are kept exactly; each
    stretch between corners is resampled at equal arc length with n = floor(L / 0.32) chords (0.32-0.64 mm of arc, sagitta <= 0.047 mm at
    the 1.1 mm smallest radius, < CHORD); collinear points are then dropped."""
    P = wire_chain(wire)
    n = len(P)
    corners = [i for i in range(n) if turn_deg(P[i - 1], P[i], P[(i + 1) % n]) > 20.0] or [0]
    out = []
    for a, b in zip(corners, corners[1:] + [corners[0] + n]):
        run = [P[i % n] for i in range(a, b + 1)]
        cum = [0.0]
        for p, q in zip(run, run[1:]): cum.append(cum[-1] + math.dist(p, q))
        L = cum[-1]
        assert L >= MIN_EDGE, f'two corners {L:.3f} mm apart at {run[0]}: a sliver survived the opening'
        k = max(1, int(L // (MIN_EDGE + 0.02)))           # arcs >= 0.32 mm so their chords stay >= MIN_EDGE
        out.append(run[0]); j = 0
        for m in range(1, k):
            t = L * m / k
            while cum[j + 1] < t: j += 1
            f = (t - cum[j]) / (cum[j + 1] - cum[j]) if cum[j + 1] > cum[j] else 0.0
            out.append((run[j][0] + f * (run[j + 1][0] - run[j][0]), run[j][1] + f * (run[j + 1][1] - run[j][1])))
    changed = True
    while changed and len(out) > 3:                      # drop collinear points (straight stretches become single edges)
        changed = False
        for i in range(len(out)):
            a, b, c = out[i - 1], out[i], out[(i + 1) % len(out)]
            ac = math.dist(a, c)
            if ac > 0 and abs((c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])) / ac < 1e-4:
                out.pop(i); changed = True; break
    bad = [(p, q) for p, q in zip(out, out[1:] + out[:1]) if math.dist(p, q) < MIN_EDGE - 1e-9]
    assert not bad, [(tuple(round(c, 3) for c in p), tuple(round(c, 3) for c in q), round(math.dist(p, q), 3)) for p, q in bad]
    return out

def offset_face(face, d):
    """Face (holes allowed) offset by d (> 0 grows it: outer wire outward, holes inward), arc joins. Returns a list of faces."""
    from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
    from OCP.GeomAbs import GeomAbs_Arc
    mo = BRepOffsetAPI_MakeOffset(face.wrapped, GeomAbs_Arc); mo.Perform(d)
    wires = sorted(cq.Shape.cast(mo.Shape()).Wires(), key=lambda w: -cq.Face.makeFromWires(w).Area())
    f = cq.Face.makeFromWires(wires[0])
    return f.cut(*[cq.Face.makeFromWires(h) for h in wires[1:]]).Faces() if len(wires) > 1 else [f]

env = poly_face([(ENV[0], ENV[1]), (ENV[2], ENV[1]), (ENV[2], ENV[3]), (ENV[0], ENV[3])])
kfaces = [inflated_face([tuple(p) for p in k['hull_xy']]) for k in keep]
raw = env.cut(*kfaces) if kfaces else env
assert len(raw.Faces()) == 1, f'keep-outs split the board into {len(raw.Faces())} pieces'
# Opening = dilate(erode(board)), with erode(env - U K) = (env shrunk by CLOSE) - U (K grown by CLOSE): booleans of convex offsets only.
shrunk = poly_face([(ENV[0] + CLOSE, ENV[1] + CLOSE), (ENV[2] - CLOSE, ENV[1] + CLOSE), (ENV[2] - CLOSE, ENV[3] - CLOSE), (ENV[0] + CLOSE, ENV[3] - CLOSE)])
grown = [cq.Face.makeFromWires(f.outerWire().offset2D(CLOSE, 'arc')[0]) for f in kfaces]
assert all(g.Area() > f.Area() for g, f in zip(grown, kfaces)), 'offset went the wrong way'
eroded = shrunk.cut(*grown) if grown else shrunk
dil = [g for f in eroded.Faces() for g in offset_face(f, CLOSE)]
opened = dil[0].fuse(*dil[1:]).clean() if len(dil) > 1 else dil[0]
res = env.intersect(opened)
faces = res.Faces()
assert len(faces) == 1, f'the opened board has {len(faces)} pieces'
removed = raw.Area() - faces[0].Area()
assert -0.01 < removed < 200, f'opening changed the board area by {removed:.1f} mm2'
def to_uv(poly):
    uv = [[round(u, 3), round(v, 3)] for u, v in (B.robot_to_board(x, y) for x, y in poly)]
    return uv if area_signed(uv) > 0 else uv[::-1]          # counter-clockwise in (u, v)
outline = to_uv(wire_polygon(faces[0].outerWire()))
cutouts = [to_uv(wire_polygon(w)) for w in faces[0].innerWires()]

def hole_ok(h):                     # inside the board, on material, and >= 4 mm from every outline / cutout edge (stricter than the +-4 mm point test)
    return inside(outline, h) and not any(inside(c, h) for c in cutouts) and min(dist_to_boundary(p, h) for p in [outline] + cutouts) >= HOLE_EDGE - 1e-6
def hole_uv(spot):
    u, v = B.robot_to_board(*spot)
    assert hole_ok((u, v)), f'robot {spot} (board {u:.2f}, {v:.2f}) is not on the board with {HOLE_EDGE} mm to every edge; the heat-set insert / standoff cannot move, so the outline or the CAD must change'
    return [round(float(u), 3), round(float(v), 3)]
holes = [hole_uv(spot) for spot in HOLE_SPOTS_ROBOT]
clamp_holes = [hole_uv(spot) for spot in CLAMP_SPOTS_ROBOT]
out = dict(outline_uv=outline, holes_uv=holes, clamp_holes_uv=clamp_holes, cutouts_uv=cutouts, keepouts_robot=keep, max_part_h=B.MAX_PART_H, height_zones=[list(z) for z in B.HEIGHT_ZONES])
(Path(__file__).parent / 'board_outline.json').write_text(json.dumps(out, indent=1))

def rbox(poly_uv):
    xs, ys = zip(*[B.board_to_robot(u, v) for u, v in poly_uv]); return f'x {min(xs):.1f}..{max(xs):.1f}, y {min(ys):.1f}..{max(ys):.1f}'
print(len(outline), 'outline points;', len(cutouts), 'cutouts;', len(keep), 'keep-outs:', [k['id'] for k in keep])
print('shell parts with no geometry in the envelope:', shells)
for k in keep: print('  keep-out', k['id'], 'hull', len(k['hull_xy']), 'pts, bounds', k['bounds'])
for c in cutouts: print('  cutout', len(c), 'pts,', rbox(c), '(robot frame)')
print('holes H1..H4 (u, v):', holes, '| robot:', [tuple(round(t, 1) for t in B.board_to_robot(*h)) for h in holes])
print('clamp holes H5, H6 (u, v):', clamp_holes, '| robot:', [tuple(round(t, 1) for t in B.board_to_robot(*h)) for h in clamp_holes],
      '| edge distance:', [round(min(dist_to_boundary(p, h) for p in [outline] + cutouts), 3) for h in clamp_holes])
print('board area %.0f mm2 (envelope %.0f, %.1f mm2 removed by the %.1f mm opening); outline in robot frame: %s' % (abs(area_signed(outline)) - sum(abs(area_signed(c)) for c in cutouts), B.BOARD_W * B.BOARD_H, removed, CLOSE, rbox(outline)))
print('shortest edge %.3f mm' % min(math.dist(p, q) for r in [outline] + cutouts for p, q in zip(r, r[1:] + r[:1])))
