import ast, json, math, unittest
from functools import lru_cache
from pathlib import Path
try: import numpy as np
except ImportError: np = None            # the mesh tests need numpy (the CAD venv has it); the JSON-only tests do not
from pcb import board_spec as B
from pcb.tests.left_out import needs, needs_kicad
OUT = Path(B.__file__).parent / 'board_outline.json'
CAD = Path(B.__file__).parents[1] / 'cad'
MANIFEST = CAD / 'component-manifest.json'
MESHES = CAD / 'assembly-meshes.json'
MARGIN = 1.5        # mm: board edge to real geometry (spec section 6)
def inside(poly, p):
    x, y = p; c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1: c = not c
    return c
def area(poly): return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]))) / 2
def edges(poly): return list(zip(poly, poly[1:] + poly[:1]))
def edge_samples(poly, step=1.0):
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        n = max(1, math.ceil(math.hypot(x2 - x1, y2 - y1) / step))
        for i in range(n): yield (x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n)

def dist_to_boundary(poly, p):
    best = 1e9
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - x1 - t * dx, p[1] - y1 - t * dy))
    return best

NECK = 2.0          # mm: narrowest board material allowed between two boundary stretches (ruling R10)
LOCAL = 3.0         # geodesic / straight-line ratio below which two close boundary points are the same corner, not a neck
def neck_violations(rings, on_board, step=0.25):
    """Pairs of boundary points closer than NECK with board material between them that are NOT a local corner.

    Sample every ring (outline and cutouts) every <= step mm. Two samples p, q with |pq| < NECK are a neck when the straight
    segment pq lies in board material (so the gap between them is material, not a notch or slot) and they are far apart along
    the boundary: on different rings, or on the same ring with boundary distance > LOCAL * |pq| (+ sampling slack). Around a
    convex board corner (even a sharp 90 deg one) the boundary distance is at most 1.42 |pq|; a thin tooth, a sliver or a narrow
    bridge between two notches gives a ratio far above 3 (a tooth with tip angle theta gives 1 / sin(theta / 2))."""
    pts, perim = [], []
    for ri, ring in enumerate(rings):
        s = 0.0
        for (x1, y1), (x2, y2) in edges(ring):
            L = math.hypot(x2 - x1, y2 - y1); n = max(1, math.ceil(L / step))
            for i in range(n): pts.append((ri, s + L * i / n, x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n))
            s += L
        perim.append(s)
    grid = {}
    for k, (_, _, x, y) in enumerate(pts): grid.setdefault((int(x // NECK), int(y // NECK)), []).append(k)
    bad = []
    for k, (ri, s, x, y) in enumerate(pts):
        cx, cy = int(x // NECK), int(y // NECK)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((cx + dx, cy + dy), ()):
                    if j <= k: continue
                    rj, sj, xj, yj = pts[j]
                    d = math.hypot(xj - x, yj - y)
                    if d >= NECK: continue
                    if ri == rj:
                        g = abs(sj - s); g = min(g, perim[ri] - g)
                        if g <= LOCAL * d + 2 * step: continue
                    if all(on_board((x + (xj - x) * t, y + (yj - y) * t)) for t in (0.25, 0.5, 0.75)):
                        bad.append(((round(x, 3), round(y, 3)), (round(xj, 3), round(yj, 3)), round(d, 3)))
    return bad

def manifest_bounds(): return {p['id']: p['bounds'] for p in json.loads(MANIFEST.read_text(encoding='utf-8'))}
def xy_centre(b): return ((b[0] + b[3]) / 2, (b[1] + b[4]) / 2)

EXTRACT_LISTS = ('CLAMP_STACK_IDS', 'SUPPORT_IDS', 'PLUGGED_CABLE_IDS', 'BOARD_CONTACT_IDS', 'NOT_OBSTACLE_IDS')
def extractor_lists():
    """B12a (carry 6): the not-obstacle id lists of extract_outline.py (the parts that stay in the CAD over or under the board but are not
    obstacles to its outline: the clamp stack, the bearing ribs and inserts, the plugged motor cables, the mated plugs and the hold-down; reasons
    there), read from its source so the ids have one home. Importing the extractor would run it (it needs CadQuery and writes
    board_outline.json), so only those five top-level assignments are run, with the manifest's {id: bounds} as its MAN."""
    ns = {'MAN': manifest_bounds()}
    for node in ast.parse((Path(B.__file__).parent / 'extract_outline.py').read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and getattr(node.targets[0], 'id', None) in EXTRACT_LISTS:
            exec(compile(ast.Module([node], []), 'extract_outline.py', 'exec'), ns)
    missing = [n for n in EXTRACT_LISTS if n not in ns]
    assert not missing, f'extract_outline.py no longer defines {missing}'
    return {n: ns[n] for n in EXTRACT_LISTS}
EXTRACT = extractor_lists()
NOT_OBSTACLES = set(EXTRACT['NOT_OBSTACLE_IDS'])
BAY_INSERTS = {i for i in EXTRACT['SUPPORT_IDS'] if i.startswith('insert_bay_')}    # heat-set inserts in P_base under the M3 holes (z 30..33)
PAR_ALL = json.loads((CAD / 'parameters.json').read_text(encoding='utf-8'))
BAY = PAR_ALL['bay_board']     # build_robot.py BAY_MOUNTS / BAY_CLAMPS as written to parameters.json
MOTORS = ('motor_left', 'motor_right')
def crossings(poly, c, axis=0):
    """Sorted crossings of the closed polygon (board frame) with the line u = c (axis 0: their v) or v = c (axis 1: their u)."""
    o = 1 - axis
    return sorted(p[o] + (c - p[axis]) * (q[o] - p[o]) / (q[axis] - p[axis]) for p, q in edges(poly) if (p[axis] > c) != (q[axis] > c))
def box_gap(a, b):
    """mm between two (u0, v0, u1, v1) boxes (0 when they touch or overlap)."""
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0.0), max(a[1] - b[3], b[1] - a[3], 0.0))

class OutlineTests(unittest.TestCase):
    def setUp(self): self.o = json.loads(OUT.read_text(encoding='utf-8'))
    def on_board(self, p): return inside(self.o['outline_uv'], p) and not any(inside(c, p) for c in self.o['cutouts_uv'])
    def at(self, x, y): return self.on_board(B.robot_to_board(x, y))
    def test_json_keys(self):
        self.assertEqual(set(self.o), {'outline_uv', 'holes_uv', 'clamp_holes_uv', 'cutouts_uv', 'keepouts_robot', 'max_part_h', 'height_zones'})
    def test_bbox_within_100mm(self):
        us, vs = zip(*self.o['outline_uv'])
        self.assertLessEqual(max(us) - min(us), 74.0 + 0.01); self.assertLessEqual(max(vs) - min(vs), 100.0 + 0.01)     # JLCPCB <= 100 x 100 price
    def test_large_enough(self):
        net = area(self.o['outline_uv']) - sum(area(c) for c in self.o['cutouts_uv'])
        self.assertGreater(net, 6000)                                        # 74 x 100 = 7400 mm2 envelope, less the two motor notches and the P03 side
        self.assertLess(net, B.BOARD_W * B.BOARD_H)
    def test_outline_inside_the_bay_envelope(self):
        for u, v in self.o['outline_uv']:
            self.assertTrue(-1e-9 <= u <= B.BOARD_W + 1e-9 and -1e-9 <= v <= B.BOARD_H + 1e-9, (u, v))
        us, vs = zip(*self.o['outline_uv'])
        self.assertAlmostEqual(min(vs), 0.0, 3); self.assertAlmostEqual(max(vs), B.BOARD_H, 3)     # the side edges are the 100 mm envelope: no part bites them
        self.assertLessEqual(min(us), 0.3); self.assertGreaterEqual(max(us), B.BOARD_W - 0.3)       # rear edge on the tray wall + 1.5 mm, front edge on the P03 - 1.5 mm
    def test_json_matches_board_spec(self):
        self.assertEqual(self.o['max_part_h'], B.MAX_PART_H)
        self.assertEqual([tuple(z) for z in self.o['height_zones']], [tuple(z) for z in B.HEIGHT_ZONES])
    def test_holes_inside_with_margin(self):
        self.assertEqual(len(self.o['holes_uv']), 4)
        for u, v in self.o['holes_uv']:
            for du, dv in ((4, 0), (-4, 0), (0, 4), (0, -4)): self.assertTrue(self.on_board((u + du, v + dv)), (u, v, du, dv))
            self.assertGreaterEqual(min(dist_to_boundary(r, (u, v)) for r in [self.o['outline_uv']] + self.o['cutouts_uv']), 4.0, (u, v))
    def test_holes_sit_on_the_bay_inserts(self):
        """The four M3 holes are on the heat-set inserts in P_base (insert_bay_0..3, in that order = H1..H4), so the inserts and the 2.4 mm spacers stay."""
        man = manifest_bounds()
        self.assertEqual(sorted(i for i in man if i.startswith('insert_bay_')), [f'insert_bay_{i}' for i in range(4)])
        ins = [xy_centre(man[f'insert_bay_{i}']) for i in range(4)]
        got = [B.board_to_robot(u, v) for u, v in self.o['holes_uv']]
        for (x, y), (gx, gy) in zip(ins, got): self.assertLess(math.hypot(x - gx, y - gy), 0.1, ((x, y), (gx, gy)))
        for (x, y), (gx, gy) in zip(BAY['mounts_mm'], got): self.assertLess(math.hypot(x - gx, y - gy), 0.1, 'BAY_MOUNTS in parameters.json')
        self.assertEqual([i for i in B.REMOVED_CAD_IDS if i.startswith('insert_')], [], 'the inserts stay')
    @needs_kicad()
    def test_h3_h4_sit_beside_the_tongue_centre(self):
        """Ruling PB4: H3 / H4 moved to robot (-11.5, +-8.5) = board (55, 41.5) / (55, 58.5), either side of the tongue's centre line, clear of the
        right-angle motor connectors J4 / J5 and their lay-flat plugs at the motor notches; H1 / H2 stay at (-19.5, +-38). B12a (carry 7): the
        tongue's edges come from the outline (its v span at the holes' u, the front edge at each hole's v), and the J4 / J5 clearance is measured:
        each hole's courtyard (B.courtyard_box) stays off both connectors' courtyards grown by their lay-flat plug (MATED_REACH toward the
        notch, as ktest_pcb's envelope)."""
        self.assertEqual([tuple(h) for h in self.o['holes_uv']], [(47.0, 12.0), (47.0, 88.0), (55.0, 41.5), (55.0, 58.5)])
        self.assertEqual([tuple(B.board_to_robot(*h)) for h in self.o['holes_uv']], [(-19.5, 38.0), (-19.5, -38.0), (-11.5, 8.5), (-11.5, -8.5)])
        step = {'+u': (1, 0), '-u': (-1, 0), '+v': (0, 1), '-v': (0, -1)}
        plugs = {}
        for ref in ('J4', 'J5'):
            (u0, v0, u1, v1), r, (du, dv) = B.courtyard_box(ref), B.MATED_REACH[ref], step[B.EDGE_CONNECTORS[ref]]
            plugs[ref] = (u0 + min(du, 0) * r, v0 + min(dv, 0) * r, u1 + max(du, 0) * r, v1 + max(dv, 0) * r)
        (u3, v3), (u4, v4) = self.o['holes_uv'][2:]
        self.assertEqual(u3, u4)
        lo, hi = crossings(self.o['outline_uv'], u3)                     # the tongue between the motor notches (one stretch of board there)
        self.assertAlmostEqual((v3 + v4) / 2, (lo + hi) / 2, 3)          # either side of the tongue's centre line
        for h, (u, v) in zip(('H3', 'H4'), self.o['holes_uv'][2:]):
            self.assertGreaterEqual(crossings(self.o['outline_uv'], v, axis=1)[-1] - u, 15.0, h)          # far from the front edge ...
            self.assertGreaterEqual(min(v - lo, hi - v), 20.0, h)                                       # ... and from the notch edges
            for ref, env in plugs.items(): self.assertGreater(box_gap(B.courtyard_box(h), env), 0.0, (h, ref))
    def test_clamp_holes_at_the_rear_jetson_standoffs(self):
        """H5 / H6: the board is clamped by the two rear Jetson standoff stacks, robot (-33, 40) and (-33, -46)."""
        self.assertEqual(len(self.o['clamp_holes_uv']), 2)
        man = manifest_bounds()
        stands = [xy_centre(man[i]) for i in ('jetson_standoff_3', 'jetson_standoff_1')]                  # BAY_CLAMPS order: left (y +40), then right (y -46)
        for (cx, cy), (x, y), (u, v) in zip(BAY['clamps_mm'], stands, self.o['clamp_holes_uv']):
            eu, ev = B.robot_to_board(cx, cy)
            self.assertAlmostEqual(u, eu, delta=0.1); self.assertAlmostEqual(v, ev, delta=0.1)
            self.assertLess(math.hypot(x - cx, y - cy), 0.1, 'standoff centre = BAY_CLAMPS')
        self.assertEqual([tuple(h) for h in self.o['clamp_holes_uv']], [(33.5, 10.0), (33.5, 96.0)])
    def test_clamp_holes_on_the_board_clear_of_the_edge(self):
        for u, v in self.o['clamp_holes_uv']:
            self.assertTrue(self.on_board((u, v)), (u, v))
            self.assertGreaterEqual(min(dist_to_boundary(r, (u, v)) for r in [self.o['outline_uv']] + self.o['cutouts_uv']), 4.0 - 1e-6, (u, v))    # the R17 4 mm hole rule
        for c in self.o['cutouts_uv']:
            for u, v in self.o['clamp_holes_uv']: self.assertFalse(inside(c, (u, v)), 'the clamp standoffs are not cutouts: they stand on the board')
    def test_keepouts_outside_outline(self):
        self.assertGreater(len(self.o['keepouts_robot']), 0)
        for k in self.o['keepouts_robot']:
            for x, y in edge_samples(k['hull_xy']):
                self.assertFalse(self.on_board(B.robot_to_board(x, y)), (k['id'], x, y))
    def test_keepouts_keep_the_margin(self):
        """Every keep-out hull edge is >= 1.5 mm from every outline / cutout edge (the board never comes closer than MARGIN)."""
        rings = [self.o['outline_uv']] + self.o['cutouts_uv']
        for k in self.o['keepouts_robot']:
            for x, y in edge_samples(k['hull_xy'], 0.5):
                p = B.robot_to_board(x, y)
                if min(dist_to_boundary(r, p) for r in rings) >= 6.0: continue
                self.assertGreaterEqual(min(dist_to_boundary(r, p) for r in rings), MARGIN - 1e-6, (k['id'], round(x, 2), round(y, 2)))
    def test_keepouts_are_the_parts_that_stay(self):
        ids = {k['id'] for k in self.o['keepouts_robot']}
        self.assertEqual(ids & set(B.REMOVED_CAD_IDS), set(), 'placeholders are not keep-outs')
        self.assertEqual(ids & NOT_OBSTACLES, set(), 'the clamp stack, bearing ribs, inserts, plugged motor cables, mated plugs and the hold-down are not keep-outs')
        self.assertTrue({'P_battery_tray', 'motor_left', 'motor_right'} <= ids, ids)          # the P03 is not listed: it sits exactly 1.5 mm off the front edge (test_front_edge_keeps_off_the_p03)
        self.assertNotIn('jetson', ids, 'the carrier is a ceiling (height zone), not a keep-out')
        self.assertEqual({i for i in ids if i.startswith(('bumper_', 'P_bumper'))}, set(), 'the floating bumper is nowhere near the board')
    def test_cutouts_are_inside_outline(self):
        for c in self.o['cutouts_uv']:
            for p in c: self.assertTrue(inside(self.o['outline_uv'], p), p)
    def test_front_edge_keeps_off_the_p03(self):
        """The P03 regulator (x 9..47, |y| <= 31) is 1.5 mm in front of the board: the tongue's front edge is the nominal x 7.5 = 9 - 1.5, and the
        board does not reach past it. B12a (carry 6): the P03's rear face and width are its p03_regulator bounds in cad/component-manifest.json."""
        x0, y0, _, _, y1, _ = manifest_bounds()['p03_regulator']
        for y in (y0 + 6.0, 0.0, y1 - 6.0):
            self.assertFalse(self.at(x0 - 1.4, y), y)
            self.assertTrue(self.at(x0 - 1.5 - 0.1, y), y)
        self.assertLessEqual(max(B.board_to_robot(u, v)[0] for u, v in self.o['outline_uv']), x0 - 1.5 + 1e-6)
    def test_not_obstacles_are_manifest_parts(self):
        """B12a (carry 6): NOT_OBSTACLES is extract_outline.py's own list (extractor_lists), and every id in it is a part of
        cad/component-manifest.json - no stale id. The mated plugs are listed by prefix (bay_plug_*, all of the manifest's): one per edge
        connector with a plug (every J of EDGE_CONNECTORS; the P03 terminals take bare wires)."""
        man = manifest_bounds()
        self.assertEqual(sorted(NOT_OBSTACLES - set(man)), [])
        plugs = {i for i in NOT_OBSTACLES if i.startswith('bay_plug_')}
        self.assertEqual(plugs, {i for i in man if i.startswith('bay_plug_')})
        self.assertEqual({i[len('bay_plug_'):] for i in plugs}, {r for r in B.EDGE_CONNECTORS if r.startswith('J')})
        self.assertEqual(BAY_INSERTS, {f'insert_bay_{i}' for i in range(4)})
    def test_rear_edge_keeps_off_the_battery_tray(self):
        """The battery tray's front wall (x -68, |y| <= 73) is 1.5 mm behind the board's nominal rear edge x -66.5."""
        for y in (-45.0, 0.0, 45.0):
            self.assertFalse(self.at(-68.0 + 1.4, y), y)
            self.assertTrue(self.at(-68.0 + 1.5 + 0.4, y), y)
    # --- ruling R10: fabricable outline ---
    def test_no_short_edges(self):
        for ring in [self.o['outline_uv']] + self.o['cutouts_uv']:
            short = [(a, b) for a, b in edges(ring) if math.dist(a, b) < 0.3]
            self.assertEqual(short, [])
    def test_no_board_neck_narrower_than_2mm(self):
        bad = neck_violations([self.o['outline_uv']] + self.o['cutouts_uv'], self.on_board)
        self.assertEqual(bad[:5], [], f'{len(bad)} neck pairs')
    def test_neck_check_itself(self):
        def run(rings): return neck_violations(rings, lambda p: inside(rings[0], p) and not any(inside(c, p) for c in rings[1:]))
        rect = [(0, 0), (20, 0), (20, 10), (0, 10)]
        self.assertEqual(run([rect]), [])                                                         # plain board: corners are local
        self.assertEqual(run([[(0, 0), (9, 0), (9, 1.5), (11, 1.5), (11, 0), (20, 0), (20, 10), (0, 10)]]), [])   # 2 mm notch = gap
        self.assertEqual(run([[(0, 0), (9, 0), (10, 4), (11, 0), (20, 0), (20, 10), (0, 10)]]), [])               # V notch = gap
        self.assertNotEqual(run([rect, [(5, 1), (15, 1), (15, 9), (5, 9)]]), [])                   # 1 mm bridge outside a cutout
        tooth = [(0, 0), (8, 0), (8, 3), (9.9, 3), (9.95, 0.2), (10.05, 0.2), (10.1, 3), (12, 3), (12, 0), (20, 0), (20, 10), (0, 10)]
        self.assertNotEqual(run([tooth]), [])                                                     # 0.1-0.2 mm tooth inside a notch
        bridge = [(0, 0), (9, 0), (9, 4), (11, 4), (11, 0), (20, 0), (20, 10), (11, 10), (11, 5.5), (9, 5.5), (9, 10), (0, 10)]
        self.assertNotEqual(run([bridge]), [])                                                    # 1.5 mm neck between two notches

# ---------------------------------------------------------------------------------------------------------------------------------
# Real-geometry tests: they read cad/assembly-meshes.json (the per-part triangle meshes the outline was cut from) and re-derive the facts
# independently of extract_outline.py.
ENV = (B.BOARD_X0, B.BOARD_Y0 - B.BOARD_H, B.BOARD_X0 + B.BOARD_W, B.BOARD_Y0)     # robot frame: xmin, ymin, xmax, ymax
Z0, Z1 = B.Z_PCB_BOTTOM - 1.0, B.Z_PCB_TOP + B.MAX_PART_H                             # the slab the board's parts can reach (1 mm under the board up to the ceiling)

@lru_cache(maxsize=None)
def cad_meshes(): return {m['id']: m for m in json.loads(MESHES.read_text(encoding='utf-8'))}
def tri_array(m):
    V = np.asarray(m['vertices'], float).reshape(-1, 3)
    return V[np.asarray(m['faces'], int).reshape(-1, 3)]
def staying_ids():
    """Parts that stay in the CAD next to / over / under the board: everything but the placeholders the board replaces, the base plate, the harness
    wires and the parts that are not obstacles (the clamp stack, the bearing ribs, the inserts, the plugged motor cables: NOT_OBSTACLES)."""
    return [i for i in cad_meshes() if i not in B.REMOVED_CAD_IDS and i != 'P_base' and not i.startswith('W') and i not in NOT_OBSTACLES]
def clamp_uv(u, v): return min(max(u, 0.0), B.BOARD_W), min(max(v, 0.0), B.BOARD_H)    # a zone that reaches a board edge continues past it
def seg_touches_rect(a, b, rect):
    """Vectorised Liang-Barsky: which of the segments a[i]-b[i] (n, 2) touch the closed rectangle."""
    d, t0, t1, ok = b - a, np.zeros(len(a)), np.ones(len(a)), np.ones(len(a), bool)
    for k, (lo, hi) in enumerate(((rect[0], rect[2]), (rect[1], rect[3]))):
        par = np.abs(d[:, k]) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            ta, tb = (lo - a[:, k]) / d[:, k], (hi - a[:, k]) / d[:, k]
        t0 = np.maximum(t0, np.where(par, -np.inf, np.minimum(ta, tb))); t1 = np.minimum(t1, np.where(par, np.inf, np.maximum(ta, tb)))
        ok &= ~(par & ((a[:, k] < lo) | (a[:, k] > hi)))
    return ok & (t1 >= t0)
def tris_touching(T, rect):
    """Mask of triangles T (n, 3, 3) whose xy projection touches the closed rectangle (edge crossing, a vertex inside, or the rectangle inside the triangle)."""
    P = T[:, :, :2]
    hit = (P[:, :, 0].max(1) >= rect[0]) & (P[:, :, 0].min(1) <= rect[2]) & (P[:, :, 1].max(1) >= rect[1]) & (P[:, :, 1].min(1) <= rect[3])
    idx = np.nonzero(hit)[0]
    if not len(idx): return hit
    Q = P[idx]; ok = np.zeros(len(idx), bool)
    for j in range(3): ok |= seg_touches_rect(Q[:, j], Q[:, (j + 1) % 3], rect)
    for cx, cy in ((rect[0], rect[1]), (rect[2], rect[1]), (rect[2], rect[3]), (rect[0], rect[3])):    # a big triangle with the rectangle wholly inside
        s = [(Q[:, (j + 1) % 3, 0] - Q[:, j, 0]) * (cy - Q[:, j, 1]) - (Q[:, (j + 1) % 3, 1] - Q[:, j, 1]) * (cx - Q[:, j, 0]) for j in range(3)]
        ok |= ((s[0] >= 0) & (s[1] >= 0) & (s[2] >= 0)) | ((s[0] <= 0) & (s[1] <= 0) & (s[2] <= 0))
    out = np.zeros(len(T), bool); out[idx[ok]] = True
    return out
def overhead_floor(rect, ids):
    """(lowest z, part id) of any surface of the given parts above the board top inside the xy rectangle. Triangles that reach below the board top
    (standoff walls, tray, mount tabs) are not 'overhead'; each triangle counts at its lowest vertex, a lower bound that is exact for the flat
    faces of the Jetson carrier, leads and lid."""
    best = (1e9, None)
    for i in ids:
        m = cad_meshes()[i]; b = m['bounds']
        if b[3] < rect[0] or b[0] > rect[2] or b[4] < rect[1] or b[1] > rect[3] or b[5] < B.Z_PCB_TOP: continue
        T = tri_array(m); zmin = T[:, :, 2].min(1)
        T, zmin = T[zmin >= B.Z_PCB_TOP - 1e-6], zmin[zmin >= B.Z_PCB_TOP - 1e-6]
        if not len(T): continue
        z = zmin[tris_touching(T, rect)]
        if len(z) and z.min() < best[0]: best = (float(z.min()), i)
    return best
def hull2d(P):
    """Convex hull (monotone chain) of an (n, 2) array, counter-clockwise, as a list of (x, y)."""
    P = sorted({(round(float(x), 6), round(float(y), 6)) for x, y in P})
    def cross(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for q in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0: lo.pop()
        lo.append(q)
    for q in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 0: up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]
def board_cells(on_board, size=1.0):
    """The board cut into size x size mm cells (board frame, aligned to the origin), the ones whose centre is on the board:
    [(robot rect, max_h, (u, v) centre)]. Each rect is shrunk by 0.01 so a part that only touches the cell's edge is not over the cell."""
    cells, e = [], 0.01
    for iu in range(int(B.BOARD_W / size)):
        for iv in range(int(B.BOARD_H / size)):
            u0, v0 = iu * size, iv * size; c = (u0 + size / 2, v0 + size / 2)
            if not on_board(c): continue
            (xa, ya), (xb, yb) = B.board_to_robot(u0, v0), B.board_to_robot(u0 + size, v0 + size)
            cells.append(((min(xa, xb) + e, min(ya, yb) + e, max(xa, xb) - e, max(ya, yb) - e), B.max_part_h_at(*c), c))
    return cells

@needs('cad/assembly-meshes.json')
@unittest.skipIf(np is None, 'needs numpy (CAD venv)')
class RealGeometryTests(unittest.TestCase):
    """Independent of extract_outline.py: sample the real meshes of every part that stays and check the written outline against them."""
    @classmethod
    def setUpClass(cls):
        cls.o = json.loads(OUT.read_text(encoding='utf-8'))
        def ring(r): return np.array([B.board_to_robot(u, v) for u, v in r])
        cls.rings = [ring(cls.o['outline_uv'])] + [ring(c) for c in cls.o['cutouts_uv']]
    def inside_np(self, ring, P):
        x, y = P[:, 0], P[:, 1]; c = np.zeros(len(P), bool)
        for (x1, y1), (x2, y2) in zip(ring, np.roll(ring, -1, 0)):
            if y1 == y2: continue
            c ^= ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1)
        return c
    def dist_to_rings(self, P):
        d = np.full(len(P), 1e9)
        for ring in self.rings:
            A, Bp = ring, np.roll(ring, -1, 0); AB = Bp - A; L2 = (AB ** 2).sum(1)
            for a, ab, l2 in zip(A, AB, L2):
                t = np.clip(((P - a) @ ab) / l2, 0, 1) if l2 else np.zeros(len(P))
                d = np.minimum(d, np.hypot(*(P - a - t[:, None] * ab).T))
        return d
    def slab_points(self, win, step=0.5, ids=None, sweep=((0.0, 0.0),)):
        """Points (robot x, y) every <= step mm along the triangle edges of the given parts (default: every staying part), inside the slab and under the
        zone ceilings; a moving part is shifted by every (dx, dy) of `sweep` (the window then applies to the shifted points)."""
        out = []
        reach = max(math.hypot(*d) for d in sweep)
        for i in (staying_ids() if ids is None else ids):
            m = cad_meshes()[i]; b = m['bounds']
            if b[3] + reach < win[0] or b[0] - reach > win[2] or b[4] + reach < win[1] or b[1] - reach > win[3] or b[5] < Z0 - 1e-3 or b[2] > Z1: continue
            T = tri_array(m)
            E = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])                    # (3n, 2, 3)
            lo, hi = E.min(1), E.max(1)
            E = E[(hi[:, 0] + reach >= win[0]) & (lo[:, 0] - reach <= win[2]) & (hi[:, 1] + reach >= win[1]) & (lo[:, 1] - reach <= win[3]) & (hi[:, 2] >= Z0 - 1e-3) & (lo[:, 2] <= Z1)]
            if not len(E): continue
            n = np.clip(np.ceil(np.linalg.norm(E[:, 1] - E[:, 0], axis=1) / step).astype(int) + 1, 2, 400)
            k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
            t = (k / np.repeat(n - 1, n))[:, None]
            e0, e1 = np.repeat(E[:, 0], n, 0), np.repeat(E[:, 1], n, 0)
            P0 = e0 + t * (e1 - e0)
            P0 = P0[(P0[:, 2] >= Z0 - 1e-3) & (P0[:, 2] <= Z1)]
            for dx, dy in sweep:
                P = P0 + np.array([dx, dy, 0.0])
                P = P[(P[:, 0] >= win[0]) & (P[:, 0] <= win[2]) & (P[:, 1] >= win[1]) & (P[:, 1] <= win[3])]
                if not len(P): continue
                uv = [clamp_uv(*B.robot_to_board(x, y)) for x, y in P[:, :2]]
                ceil = np.array([B.Z_PCB_TOP + B.max_part_h_at(u, v) for u, v in uv])
                P = P[P[:, 2] <= ceil + 1e-6]
                if len(P): out.append((i, P[:, :2]))
        return out
    def clear_of_board(self, P):
        """Distance of each point (n, 2) to the board: 0 on it, else to the nearest outline / cutout edge."""
        on = self.inside_np(self.rings[0], P)
        for c in self.rings[1:]: on &= ~self.inside_np(c, P)
        return np.where(on, 0.0, self.dist_to_rings(P))
    def test_every_part_that_stays_keeps_1p5mm_off_the_board(self):
        """The spec's >= 1.5 mm to real geometry: no sampled point of any remaining part, at a height a board part could reach, is on the board
        or closer than 1.5 mm to it. This covers the motors, the P03, the battery tray and the battery straps."""
        win = (ENV[0] - 3, ENV[1] - 3, ENV[2] + 3, ENV[3] + 3)
        seen = {}
        for i, P in self.slab_points(win):
            d = self.clear_of_board(P)
            seen[i] = (len(P), round(float(d.min()), 3))
            self.assertGreaterEqual(float(d.min()), MARGIN - 1e-6, (i, seen[i]))
        self.assertTrue({'P_battery_tray', 'motor_left', 'motor_right', 'p03_regulator'} <= set(seen), sorted(seen))
        self.assertNotIn('jetson', seen, 'the carrier is above the height-zone ceiling')
    def test_not_obstacles_stand_on_the_board(self):
        """B12a (carry 6, replaces a check that the clamp stack and the ribs were not among the staying parts, which staying_ids made true by
        construction): every NOT_OBSTACLES part but the heat-set inserts really reaches into the slab on the board (sampled as above, it is
        on the board), so leaving it out is what lets the 1.5 mm check pass - the list holds no part that could be a plain keep-out. The
        inserts (z 30..33) sit under this slab, touching only the extractor's slab floor (z 33, the P_base top); test_holes_sit_on_the_bay_inserts
        places them."""
        win = (ENV[0] - 3, ENV[1] - 3, ENV[2] + 3, ENV[3] + 3)
        on = {i for i, P in self.slab_points(win, ids=sorted(NOT_OBSTACLES)) if float(self.clear_of_board(P).min()) == 0.0}
        self.assertEqual(NOT_OBSTACLES - on, BAY_INSERTS)
    def test_motor_notches(self):
        """The motors (x +-12.5, |y| >= 35) bite the board's two front-side corners: no outline point (or cutout point) within 1.5 mm of either motor's xy hull."""
        for sign, motor in ((1, 'motor_left'), (-1, 'motor_right')):
            V = np.asarray(cad_meshes()[motor]['vertices'], float).reshape(-1, 3)
            H = hull2d(V[:, :2])
            self.assertAlmostEqual(min(abs(y) for _, y in H), 35.0, 3, motor)
            pts = [B.board_to_robot(u, v) for ring in [self.o['outline_uv']] + self.o['cutouts_uv'] for u, v in edge_samples(ring, 0.25)]
            d = min(0.0 if inside(H, p) else dist_to_boundary(H, p) for p in pts)
            self.assertGreaterEqual(d, MARGIN - 1e-6, (motor, round(d, 3)))
            def at(x, y): return self.inside_np(self.rings[0], np.array([[x, sign * y]]))[0]
            self.assertFalse(at(0.0, 34.5), motor)                 # the notch: the tongue is narrower than the full width
            self.assertTrue(at(0.0, 32.0), motor)                  # the tongue itself
            self.assertTrue(at(-20.0, 45.0), motor)                # behind the motor the board is full width
    def test_bumper_parts_swept_stay_clear_of_the_board(self):
        """The floating bumper travels 5 mm along its axis. Each bumper part (skins, riders, pins, races), swept along that travel, is >= 1.5 mm from
        the board outline wherever it is in the slab."""
        travel, axis = PAR_ALL['bumper_travel'], math.radians(PAR_ALL['bumper_axis_deg'])
        bump = [i for i in cad_meshes() if i.startswith(('bumper_', 'P_bumper'))]
        self.assertGreaterEqual(len([i for i in bump if i.startswith('P_bumper_rider_')]), 6)
        win = (ENV[0] - 3, ENV[1] - 3, ENV[2] + 3, ENV[3] + 3)
        for i in bump:
            b = cad_meshes()[i]['bounds']
            sgn = 1 if (b[1] + b[4]) / 2 > 0 else -1                                               # left half (y > 0) travels (-cos, -sin), right half (-cos, +sin)
            d = (-math.cos(axis), -sgn * math.sin(axis))
            for _, P in self.slab_points(win, ids=[i], sweep=[(s * travel * d[0], s * travel * d[1]) for s in np.linspace(0, 1, 6)]):
                self.assertGreaterEqual(float(self.clear_of_board(P).min()), MARGIN - 1e-6, i)

@needs('cad/assembly-meshes.json')
@unittest.skipIf(np is None, 'needs numpy (CAD venv)')
class BoardHeightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.o = json.loads(OUT.read_text(encoding='utf-8'))
    def on_board(self, p): return inside(self.o['outline_uv'], p) and not any(inside(c, p) for c in self.o['cutouts_uv'])
    def test_height_zones_cover_the_area_under_the_jetson_carrier(self):
        """Real meshes: every board cell under the Jetson carrier's xy hull is in the under-carrier zone (u >= CARRIER_U, x >= -38.5), the zone is the
        carrier footprint plus a 1.5 mm margin at its rear, and the zone limit leaves >= 1.5 mm to the carrier's lowest leads."""
        J = tri_array(cad_meshes()['jetson'])
        V = J.reshape(-1, 3); carrier = V[V[:, 2] <= 60.0]                          # PCB, its leads and the connector bodies, not the heatsink
        H = hull2d(carrier[:, :2])
        cells = [(c, lim) for _, lim, c in board_cells(self.on_board) if inside([B.robot_to_board(x, y) for x, y in H], c)]
        self.assertGreater(len(cells), 500)
        for (u, v), lim in cells:
            self.assertGreaterEqual(u, B.CARRIER_U, (u, v)); self.assertEqual(lim, B.UNDER_CARRIER_MAX_H, (u, v))
        rear_u = B.robot_to_board(float(carrier[:, 0].min()), 0.0)[0]
        self.assertAlmostEqual(rear_u - B.CARRIER_U, 1.5, 1, 'the zone starts 1.5 mm behind the carrier edge')
        (zu0, zv0, zu1, zv1, zh), = B.HEIGHT_ZONES
        self.assertGreaterEqual(zu1, B.BOARD_W); self.assertLessEqual(zv0, 0.0); self.assertGreaterEqual(zv1, B.BOARD_H)     # to the front and both side edges, as the carrier (y +-50) does
        floor = overhead_floor((ENV[0] + zu0 + 0.01, ENV[1], ENV[2], ENV[3]), ['jetson'])           # the carrier's lowest surface over the board (z 55.57)
        self.assertGreaterEqual(float(J[:, :, 2].min()), B.UNDER_CARRIER_Z + 0.3 - 1e-6)           # and anywhere (z 55.07, ahead of the board): the zone is sized to z 54.77, 0.3 mm under it
        self.assertGreaterEqual(floor[0], B.UNDER_CARRIER_Z + 0.3 - 1e-6)
        self.assertGreaterEqual(floor[0] - B.Z_PCB_TOP - zh, B.CARRIER_CLEARANCE - 1e-6)          # zone limit leaves >= 1.5 mm to the lowest lead
    def test_every_cell_of_the_board_is_under_its_height_limit(self):
        """Cut the board into 1 mm cells; for each, board top + limit is <= the lowest surface of any staying part above the cell - 1.5 mm
        (or <= z 97.5, lid underside - 3, where nothing is above). Only surfaces above the board top count; the clamp stack, ribs and plugged cables are not overhead."""
        ids = staying_ids()
        cells = board_cells(self.on_board)
        self.assertGreater(len(cells), 6000)
        T, zmin = [], []
        win = (ENV[0], ENV[1], ENV[2], ENV[3])
        for i in ids:
            m = cad_meshes()[i]; b = m['bounds']
            if b[3] < win[0] or b[0] > win[2] or b[4] < win[1] or b[1] > win[3] or b[5] < B.Z_PCB_TOP: continue
            t = tri_array(m); z = t[:, :, 2].min(1)
            k = (z >= B.Z_PCB_TOP - 1e-6) & (z <= B.Z_PCB_TOP + B.MAX_PART_H + B.CARRIER_CLEARANCE + 1e-6)      # higher surfaces cannot bind a limit <= 97.5
            T.append(t[k]); zmin.append(z[k])
        T, zmin = np.concatenate(T), np.concatenate(zmin)
        lo_xy, hi_xy = T[:, :, :2].min(1), T[:, :, :2].max(1)
        lowest = (1e9, None)
        for rect, lim, c in cells:
            near = (hi_xy[:, 0] >= rect[0]) & (lo_xy[:, 0] <= rect[2]) & (hi_xy[:, 1] >= rect[1]) & (lo_xy[:, 1] <= rect[3])
            idx = np.nonzero(near)[0]
            floor = None
            if len(idx):
                hit = idx[tris_touching(T[idx], rect)]
                if len(hit): floor = float(zmin[hit].min())
            ceiling = B.CEILING_Z_LID if floor is None else floor - B.CARRIER_CLEARANCE
            self.assertLessEqual(B.Z_PCB_TOP + lim, ceiling + 1e-6, (c, lim, floor))
            self.assertLessEqual(B.Z_PCB_TOP + lim, B.CEILING_Z_LID + 1e-6, c)
            if floor is not None and floor < lowest[0]: lowest = (floor, c)
        self.assertLess(lowest[0], 1e9, 'the Jetson carrier is over the board')

if __name__ == '__main__':
    unittest.main()
