"""Placed-board checks (Task 5, redone for the low-profile strip in L5). Run with KiCad's Python:
    cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_pcb -v"""
import json, math, re, shutil, tempfile, unittest
from pathlib import Path
import pcbnew
from pcb import board_spec as B, copper as C, zone_check
D = Path(B.__file__).parent
PCB, PRO, DRU = D / 'r1_power_io.kicad_pcb', D / 'r1_power_io.kicad_pro', D / 'r1_power_io.kicad_dru'
LAYER = {L: getattr(pcbnew, L.replace('.', '_')) for L in C.STACKUP}      # STACKUP name -> pcbnew layer id (F_Cu, In1_Cu, ...)
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
OUTLINE = json.loads((D / 'board_outline.json').read_text(encoding='utf-8'))


def polyset(pts):
    ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
    for u, v in pts: ps.Append(MM(u), MM(v))
    return ps


def area_mm2(ps): return ps.Area() / 1e12 if ps.OutlineCount() else 0.0


def minus(a, b):
    r = a.CloneDropTriangulation(); r.BooleanSubtract(b); return r


def both(a, b):
    r = a.CloneDropTriangulation(); r.BooleanIntersection(b); return r


def board_poly():
    """Board material: outline minus the cutouts."""
    ps = polyset(OUTLINE['outline_uv'])
    for c in OUTLINE['cutouts_uv']: ps.BooleanSubtract(polyset(c))
    return ps


def fab_body(fp):
    """The module outline rectangle that build_pcb draws on F.Fab, as a board-frame polygon (None if the footprint has none)."""
    for item in fp.GraphicalItems():
        if item.GetLayer() == pcbnew.F_Fab and item.GetClass() == 'PCB_SHAPE' and item.GetShape() == pcbnew.SHAPE_T_RECT and abs(TOMM(item.GetWidth()) - 0.1) < 1e-6:
            ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, pcbnew.F_Fab, 0, MM(0.001), pcbnew.ERROR_INSIDE)
            return ps
    return None


def box(ps):
    """(u0, v0, u1, v1) mm of a polygon set."""
    bb = ps.BBox()
    return (TOMM(bb.GetX()), TOMM(bb.GetY()), TOMM(bb.GetRight()), TOMM(bb.GetBottom()))


def effective_h(ref):
    """mm a part stands over the board top: its body (PARTS h) or, for an edge connector, its mated plug plus the wire bend over a
    vertical part (MATED_H + CABLE_BEND_MM), whichever is taller."""
    h = B.PARTS[ref]['h']
    if ref in B.EDGE_CONNECTORS:
        fam = B.connector_family(ref)
        h = max(h, B.MATED_H[ref] + (B.CABLE_BEND_MM[fam] if B.PARTS[ref]['fp'].endswith('_Vertical') else 0.0))
    return h


_FPS = {}           # ref -> footprint of the saved board (filled by PcbTests.setUpClass)
_DIRS = {'+u': (1, 0), '-u': (-1, 0), '+v': (0, 1), '-v': (0, -1)}


def lane_box(board, ref, direction):
    """The cable lane of an edge connector, (u0, v0, u1, v1): its courtyard's width, from the courtyard's side facing `direction`, swept
    along `direction` until the lane's centre line leaves the board outline `board` (a SHAPE_POLY_SET)."""
    u0, v0, u1, v1 = box(_FPS[ref].GetCourtyard(pcbnew.F_CrtYd))
    du, dv = _DIRS[direction]
    su = {-1: u0, 1: u1, 0: (u0 + u1) / 2}[du]; sv = {-1: v0, 1: v1, 0: (v0 + v1) / 2}[dv]
    L = 0.0
    while L < 120 and board.Contains(pcbnew.VECTOR2I(MM(su + du * L), MM(sv + dv * L))): L += 0.05
    if du: return (min(su, su + du * L), v0, max(su, su + du * L), v1)
    return (u0, min(sv, sv + dv * L), u1, max(sv, sv + dv * L))


def cable_lane_obstacles(board, ref, direction, max_h=3.0, plug_only=False):
    """Refs (sorted) whose courtyard or module body enters the cable lane of `ref` (lane_box) and stand taller than max_h (effective_h).
    plug_only: the lane is only the lay-flat plug's footprint past the courtyard (MATED_REACH long)."""
    lu0, lv0, lu1, lv1 = lane_box(board, ref, direction)
    if plug_only:
        r = B.MATED_REACH[ref]
        du, dv = _DIRS[direction]
        if du > 0: lu1 = lu0 + r
        elif du < 0: lu0 = lu1 - r
        elif dv > 0: lv1 = lv0 + r
        else: lv0 = lv1 - r
    lane = polyset(B.rect_pts((lu0 + 0.01, lv0 + 0.01, lu1 - 0.01, lv1 - 0.01)))
    hit = []
    for other, fp in _FPS.items():
        if other == ref or effective_h(other) <= max_h: continue
        shapes = [fp.GetCourtyard(pcbnew.F_CrtYd)] + ([fab_body(fp)] if fab_body(fp) is not None else [])
        if any(area_mm2(both(lane, s)) > 1e-4 for s in shapes): hit.append(other)
    return sorted(hit)


def patch_problems(board):
    """(spokes, vias) on a filled board - ruling PB21 (B6 fix 2): every F.Cu GND patch (board_spec.GND_PATCHES, zone GND_PATCH_<k>)
    joins its pin (board_spec.GND_PATCH_PINS) through real spokes - fill that reaches the pad (zone_check._spokes_into), not fill that
    only crosses the relief ring - and every GND stitching via inside a patch ties live copper on >= 2 layers (zone_check
    .via_copper_layers: the trunk, or a piece a pad touches). spokes: [(zone, pin, n)] for a patch that gives its pin no spoke; vias:
    [((u, v), layers)] for a stitching via with fewer than 2; held: {zone: stitching vias inside it}."""
    spokes, vias, held = [], [], {}
    zones = {z.GetZoneName(): z for z in board.Zones() if not z.GetIsRuleArea()}
    on_board = {(round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)) for v in board.GetTracks()
                if v.GetClass() == 'PCB_VIA' and v.GetNetname() == 'GND'}
    for k, (ref, pin) in enumerate(B.GND_PATCH_PINS):
        z = zones[f'GND_PATCH_{k + 1}']
        fill = zone_check.solid_fill(z, pcbnew.F_Cu)
        gap, zw = TOMM(z.GetThermalReliefGap()), TOMM(z.GetThermalReliefSpokeWidth())
        n = sum(zone_check._spokes_into(p, pcbnew.F_Cu, fill, gap, zw) for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == pin)
        if n < 1: spokes.append((z.GetZoneName(), f'{ref}.{pin}', n))
        held[z.GetZoneName()] = [uv for net, uv in B.STITCH_VIAS if net == 'GND' and uv in on_board
                                 and fill.Contains(pcbnew.VECTOR2I(MM(uv[0]), MM(uv[1])))]
        for uv in held[z.GetZoneName()]:
            own = zone_check.via_copper_layers(board, 'GND', uv)
            if len(own) < 2: vias.append((uv, own))
    return spokes, vias, held


class PcbTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = pcbnew.LoadBoard(str(PCB))
        cls.fps = {fp.GetReference(): fp for fp in cls.b.GetFootprints()}
        cls.board = board_poly()
        _FPS.clear(); _FPS.update(cls.fps)
        cls._filled, cls._tmp = None, None

    @classmethod
    def tearDownClass(cls):
        if cls._tmp is not None: cls._tmp.cleanup()

    @classmethod
    def filled(cls):
        """A scratch copy of the saved board (with its project and DRC rules) with every zone filled, made once per run."""
        if cls._filled is None:
            cls._tmp = tempfile.TemporaryDirectory()
            for f in (PCB, PRO, DRU): shutil.copy(f, Path(cls._tmp.name) / f.name)
            cls._filled = pcbnew.LoadBoard(str(Path(cls._tmp.name) / PCB.name))
            pcbnew.ZONE_FILLER(cls._filled).Fill(cls._filled.Zones())
        return cls._filled

    # --- brief, Task 5 Step 1 ---
    def test_every_pad_has_its_spec_net(self):
        want = {p: n for n, pins in B.NETS.items() for p in pins}
        got = {(fp.GetReference(), pad.GetNumber()): pad.GetNetname() for fp in self.b.GetFootprints() for pad in fp.Pads()}
        for key, net in want.items(): self.assertEqual(got.get(key), net, key)

    def test_all_parts_placed(self):
        self.assertEqual({fp.GetReference() for fp in self.b.GetFootprints()}, set(B.PARTS))

    def test_courtyards_inside_outline(self):
        edge = self.b.GetBoardEdgesBoundingBox()
        for fp in self.b.GetFootprints():
            bb = fp.GetBoundingBox(False)
            self.assertTrue(edge.Contains(bb.GetOrigin()) and edge.Contains(bb.GetEnd()), fp.GetReference())

    # --- stronger geometry checks ---
    def test_no_net_on_unlisted_pads(self):
        """A pad off every board_spec net has no net - except a no-connect pin (board_spec.NC), which carries KiCad's own single-pin
        'unconnected-(...)' net from the schematic netlist (B7, schematic parity), that pin alone."""
        want = {p for pins in B.NETS.values() for p in pins}
        pads = [(fp.GetReference(), pad.GetNumber(), pad.GetNetname()) for fp in self.b.GetFootprints() for pad in fp.Pads()]
        extra = [(r, n, net) for r, n, net in pads if net and (r, n) not in want and not ((r, n) in B.NC and net.startswith('unconnected-('))]
        self.assertEqual(extra, [])
        nc = {(r, n): net for r, n, net in pads if (r, n) in B.NC}
        self.assertEqual(set(nc), set(B.NC))
        self.assertTrue(all(net.startswith(f'unconnected-({r}-') for (r, n), net in nc.items()), nc)
        self.assertEqual(len(set(nc.values())), len(nc))

    def test_courtyards_on_board_material(self):
        """Every courtyard (module bodies included) lies on the board: not past the outline, not in a notch or a cutout."""
        for ref, fp in self.fps.items():
            out = area_mm2(minus(fp.GetCourtyard(pcbnew.F_CrtYd), self.board))
            self.assertLess(out, 0.01, f'{ref}: {out:.2f} mm2 of courtyard off the board')

    def test_courtyards_do_not_overlap(self):
        refs = sorted(self.fps)
        cy = {r: self.fps[r].GetCourtyard(pcbnew.F_CrtYd) for r in refs}
        bad = [(a, b, round(area_mm2(both(cy[a], cy[b])), 2)) for i, a in enumerate(refs) for b in refs[i + 1:]
               if self.fps[a].GetBoundingBox(False).Intersects(self.fps[b].GetBoundingBox(False)) and area_mm2(both(cy[a], cy[b])) > 1e-4]
        self.assertEqual(bad, [])

    def test_module_bodies_on_board(self):
        for ref in (r for m in B.MODULES.values() for r in m['rows']):
            body = fab_body(self.fps[ref]); self.assertIsNotNone(body, ref)
            self.assertLess(area_mm2(minus(body, self.board)), 0.01, ref)

    def test_dsn_export_keepouts_are_only_the_cutouts(self):
        """Export the Specctra DSN (what Freerouting reads) from a temp copy: the board has no rule areas, and every structure-level
        keepout of the export is one of the outline's cutouts (a KiCad rule area would come out as a router keepout). The bay board
        (B2) has no cutouts, so the export has no structure-level keepout at all."""
        from pcb.sexpr import parse, find
        self.assertEqual([z.GetZoneName() for z in self.b.Zones() if z.GetIsRuleArea()], [])
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, PRO): shutil.copy(f, Path(t) / f.name)
            b = pcbnew.LoadBoard(str(Path(t) / PCB.name))
            dsn = Path(t) / 'export.dsn'
            self.assertTrue(pcbnew.ExportSpecctraDSN(b, str(dsn)))
            root = parse(dsn.read_text(encoding='utf-8'))[0]
        (structure,) = find(root, 'structure')
        self.assertEqual(find(root, 'resolution')[0][1], 'um')                 # coordinates below are in micrometres
        cut = pcbnew.SHAPE_POLY_SET()
        for c in OUTLINE['cutouts_uv']: cut.Append(polyset(c))
        cut.Inflate(MM(0.3), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
        keepouts = find(structure, 'keepout')
        self.assertEqual(len(keepouts) > 0, bool(OUTLINE['cutouts_uv']), 'structure-level keepouts must be exactly the cutouts')
        for k in keepouts:
            shapes = [c for c in k if isinstance(c, list) and c and c[0] not in ('clearance_class', 'window')]
            self.assertEqual(len(shapes), 1, f'keepout with {len(shapes)} shapes: {k}')
            for shape in shapes:                                  # fix 1: any shape other than polygon / rect fails, none is skipped
                self.assertIn(shape[0], ('polygon', 'rect'), f'unrecognised DSN keepout shape {shape[0]}')
                nums = [float(x) / 1000.0 for x in shape[2:]]         # DSN unit um, y axis up: board (u, v) = (x, -y)
                if shape[0] == 'polygon': pts = [(nums[i], -nums[i + 1]) for i in range(1, len(nums) - 1, 2)]
                else: x0, y0, x1, y1 = nums; pts = [(x0, -y0), (x1, -y0), (x1, -y1), (x0, -y1)]
                self.assertLess(area_mm2(minus(polyset(pts), cut)), 0.05, f'DSN keepout on {shape[1]} is not a cutout')

    def test_mounting_holes_at_outline_holes(self):
        for i, (u, v) in enumerate(OUTLINE['holes_uv'], 1):
            p = self.fps[f'H{i}'].GetPosition()
            self.assertEqual(B.PLACEMENT[f'H{i}'], (u, v, 0), f'H{i}')
            self.assertAlmostEqual(TOMM(p.x), u, 3); self.assertAlmostEqual(TOMM(p.y), v, 3)

    def test_edge_cuts_has_outline_and_cutouts(self):
        ps = pcbnew.SHAPE_POLY_SET()
        self.assertTrue(self.b.GetBoardPolygonOutlines(ps))
        self.assertEqual(ps.OutlineCount(), 1)
        self.assertEqual(ps.HoleCount(0), len(OUTLINE['cutouts_uv']))
        self.assertAlmostEqual(area_mm2(ps), area_mm2(self.board), delta=1.0)

    # --- ruling R12: net classes from r1_power_io.kicad_pro apply to the saved board ---
    def test_net_classes_apply_on_load(self):
        for net, cls in (('BAT+', 'Power'), ('F2_OUT', 'Power'), ('VMOT', 'PowerTap'), ('5V', 'PowerTap'), ('L_M1', 'Motor'),
                         ('R_M2', 'Motor'), ('SDA', 'Default')):
            self.assertEqual(self.b.FindNet(net).GetNetClassName(), cls, net)

    def test_project_file_keeps_net_classes(self):
        ns = json.loads(PRO.read_text(encoding='utf-8'))['net_settings']
        classes = {c['name']: c for c in ns['classes']}
        pats = {(p['netclass'], p['pattern']) for p in ns['netclass_patterns']}
        for name, c in B.NETCLASSES.items():
            self.assertAlmostEqual(classes[name]['track_width'], c['width']); self.assertAlmostEqual(classes[name]['clearance'], c['clearance'])
            for n in c['nets']: self.assertIn((name, n), pats)

    # --- bay B4: 4-layer stack-up, In1 GND plane, In2 power pours, outer-layer trunks (ruling R4: IPC-2221 widths, copper.py) ---
    def zones(self, layer):
        """{net: zone} of the copper zones on `layer` (a copper.STACKUP name); one zone per net and layer."""
        out = {}
        for z in self.b.Zones():
            if z.GetIsRuleArea() or z.GetLayer() != LAYER[layer]: continue
            if layer == 'F.Cu' and z.GetNetname() == 'GND': continue        # fix 1: the GND patches, one zone each (test_power_zones_present)
            self.assertNotIn(z.GetNetname(), out, f'two {z.GetNetname()} zones on {layer}')
            out[z.GetNetname()] = z
        return out

    def test_board_is_the_ordered_4_layer_stackup(self):
        """Bay B4: four copper layers in copper.STACKUP order, 1.6 mm thick, and the board's stack-up (KiCad's BOARD_STACKUP, saved in the
        board file's (setup (stackup ...))) lists the copper layers in that order at COPPER_OZ x 0.035 mm each, summing to 1.6 mm with the
        dielectrics and masks."""
        from pcb.sexpr import parse, find
        self.assertEqual(self.b.GetCopperLayerCount(), 4)
        self.assertTrue(all(self.b.IsLayerEnabled(LAYER[L]) for L in C.STACKUP))
        self.assertAlmostEqual(TOMM(self.b.GetDesignSettings().GetBoardThickness()), B.PCB_T, 4)
        (setup,) = find(parse(PCB.read_text(encoding='utf-8'))[0], 'setup')
        (stack,) = find(setup, 'stackup')
        items = [(str(l[1]), {c[0]: c[1] for c in l[2:] if isinstance(c, list) and len(c) > 1}) for l in find(stack, 'layer')]
        copper = [(n, float(d['thickness'])) for n, d in items if d.get('type') == 'copper']
        self.assertEqual([n for n, _ in copper], list(C.STACKUP))
        for n, t in copper: self.assertAlmostEqual(t, C.OZ_MM * C.COPPER_OZ[n], 6, n)
        self.assertAlmostEqual(sum(float(d['thickness']) for _, d in items if 'thickness' in d), B.PCB_T, 4)
        # bay B5 (hand-over 1): In1.Cu is a plane - layer type power, which the Specctra export passes on so Freerouting never routes it
        self.assertEqual({L: self.b.GetLayerType(LAYER[L]) for L in C.STACKUP},
                         {L: (pcbnew.LT_POWER if L == C.GND_PLANE_LAYER else pcbnew.LT_SIGNAL) for L in C.STACKUP})

    def test_power_zones_present(self):
        """The zones exist (build_pcb saves them unfilled; route.py fills the routed board and ktest_route checks that fill): In1.Cu holds
        exactly one zone, the GND plane over the whole outline at priority 0 with 0.2 mm clearance and 0.2 mm minimum width (the webs
        between 2.54 mm pins survive); In2.Cu holds the VBAT_SW, VMOT, 5V and 3V3 pours; B.Cu a GND fill over the whole outline at
        priority 0 under its POWER_ZONES (each at a higher priority); F.Cu its POWER_ZONES plus one GND zone per GND_PATCHES polygon (fix 1,
        ruling PB12). Every zone is its POWER_ZONES / GND_PATCHES polygon."""
        self.assertEqual(set(self.zones('In1.Cu')), {'GND'})
        in1 = self.zones('In1.Cu')['GND']
        self.assertLess(area_mm2(minus(self.board, in1.Outline())), 0.01, 'In1 GND plane does not cover the board')
        self.assertEqual(in1.GetAssignedPriority(), 0)
        self.assertAlmostEqual(TOMM(in1.GetLocalClearance()), 0.2, 4); self.assertAlmostEqual(TOMM(in1.GetMinThickness()), 0.2, 4)
        want = {L: {z['net'] for z in B.POWER_ZONES if L in z['layers']} for L in ('F.Cu', 'In2.Cu', 'B.Cu')}
        self.assertLessEqual({'VBAT_SW', 'VMOT', '5V', '3V3'}, want['In2.Cu'])
        self.assertEqual(set(self.zones('F.Cu')), want['F.Cu'])
        patches = [z for z in self.b.Zones() if z.GetLayer() == pcbnew.F_Cu and z.GetNetname() == 'GND' and not z.GetIsRuleArea()]
        self.assertEqual(len(patches), len(B.GND_PATCHES))                  # fix 1 (PB12): one F.Cu GND zone per patch
        for z, pts in zip(sorted(patches, key=lambda z: z.GetAssignedPriority()), B.GND_PATCHES):
            self.assertLess(area_mm2(minus(z.Outline(), polyset(pts))) + area_mm2(minus(polyset(pts), z.Outline())), 0.01, z.GetZoneName())
        self.assertEqual(set(self.zones('In2.Cu')), want['In2.Cu'])
        self.assertEqual(set(self.zones('B.Cu')), want['B.Cu'] | {'GND'})
        fill = self.zones('B.Cu')['GND']
        self.assertLess(area_mm2(minus(self.board, fill.Outline())), 0.01, 'B.Cu GND fill does not cover the board')
        self.assertEqual(fill.GetAssignedPriority(), 0)
        for L in ('F.Cu', 'In2.Cu', 'B.Cu'):
            got = self.zones(L)
            for z in (z for z in B.POWER_ZONES if L in z['layers']):
                outline, poly = got[z['net']].Outline(), polyset(z['poly'])
                self.assertLess(area_mm2(minus(outline, poly)) + area_mm2(minus(poly, outline)), 0.01, (z['net'], L))
                if L == 'B.Cu': self.assertGreater(got[z['net']].GetAssignedPriority(), fill.GetAssignedPriority(), z['net'])

    def test_power_zone_polygons_disjoint_and_on_board(self):
        """Per layer, the POWER_ZONES polygons lie on the board and keep 0.5 mm from each other."""
        for L in ('F.Cu', 'In2.Cu', 'B.Cu'):
            polys = {z['net']: polyset(z['poly']) for z in B.POWER_ZONES if L in z['layers']}
            names = sorted(polys)
            for i, a in enumerate(names):
                self.assertLess(area_mm2(minus(polys[a], self.board)), 0.01, f'{a} zone on {L} off the board / over a cutout')
                for b in names[i + 1:]:
                    gap = polys[a].CloneDropTriangulation(); gap.Inflate(MM(0.25), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
                    self.assertLess(area_mm2(both(gap, polys[b])), 1e-6, f'{a} and {b} zones on {L} closer than 0.5 mm')

    def test_power_zones_cover_trunk_pads_with_3mm_corridor(self):
        """Each trunk polygon (POWER_ZONES entry without trunk=False) contains all its POWER_TRUNK pads, and the polygon eroded by
        zone_check.ERODE (a >= 3 mm corridor, ruling R4) is a single piece that comes within 2.2 mm of every trunk pad centre. Bay B4:
        the 2-layer rule asked 3.9 mm of polygon (the 3 mm corridor plus 0.45 mm of clearance on each side, the fill being the polygon
        less the clearances); the 4-layer polygons are drawn on the clearances themselves where they wind between fixed copper (the 5V
        strip's octagon notches round the LP12 GND vias in the driver interiors leave 3.1-3.3 mm), so the polygon is held to the trunk
        corridor itself and test_filled_copy_keeps_every_trunk_connected checks that corridor on the fill."""
        from pcb import zone_check
        for z in (z for z in B.POWER_ZONES if z.get('trunk', True)):
            net, ps = z['net'], polyset(z['poly'])
            core = ps.CloneDropTriangulation(); core.Deflate(MM(zone_check.ERODE), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
            self.assertEqual(core.OutlineCount(), 1, f'{net} {z["layers"]}: corridor narrower than 3 mm somewhere')
            for ref, num in B.POWER_TRUNK[net]:
                pad = [p for p in self.fps[ref].Pads() if p.GetNumber() == num][0]
                c = pad.GetPosition()
                self.assertTrue(ps.Contains(c), f'{net}: {ref}.{num} outside its zone')
                self.assertLessEqual(TOMM(int(math.sqrt(core.SquaredDistance(c)))), 2.2, f'{net}: {ref}.{num} far from the corridor')

    def test_power_zone_pads_use_wide_thermal_reliefs(self):
        """Ruling R13: every pad joined to a zone on any layer (In1 GND plane, B.Cu GND fill, the F.Cu / In2 / B.Cu power zones) gets a
        thermal relief with wide spokes (zone setting: 4 spokes of >= 1.2 mm, gap 0.3-0.5 mm), never a solid connection; no pad
        overrides it."""
        zs = [z for z in self.b.Zones() if not z.GetIsRuleArea()]
        self.assertEqual({z.GetLayer() for z in zs}, {LAYER[L] for L in C.STACKUP})
        for z in zs:
            key = (z.GetNetname(), self.b.GetLayerName(z.GetLayer()))
            self.assertEqual(z.GetPadConnection(), pcbnew.ZONE_CONNECTION_THERMAL, key)
            self.assertGreaterEqual(TOMM(z.GetThermalReliefSpokeWidth()), 1.2, key)
            self.assertTrue(0.3 <= TOMM(z.GetThermalReliefGap()) <= 0.5, key)
        nets = {z.GetNetname() for z in zs}
        for fp in self.b.GetFootprints():
            for p in fp.Pads():
                if p.GetNetname() in nets:
                    self.assertEqual(p.GetLocalZoneConnection(), pcbnew.ZONE_CONNECTION_INHERITED, (fp.GetReference(), p.GetNumber()))

    def test_filled_copy_keeps_every_trunk_connected(self):
        """R4 restated for thermal reliefs, per net and per layer it is a trunk on (zone_check.trunk_layers), on a freshly filled scratch
        copy of the board: see pcb/zone_check.py (shared with ktest_route and the B6 power check). Every POWER_TRUNK net has its zones
        (no trunk net is zoneless). On each trunk layer the zone copper eroded by 1.5 mm keeps one piece (GND: the largest) that every
        POWER_TRUNK pad of the net reaches through >= 2 relief spokes, within pad radius + relief gap + 1.5 mm + 0.5 mm."""
        self.assertEqual([n for n in B.POWER_TRUNK if not zone_check.trunk_layers(n)], [], 'zoneless trunk nets')
        b = self.filled()
        rep = zone_check.trunk_report(b)
        self.assertEqual(set(rep), {(n, L) for n in B.POWER_TRUNK for L in zone_check.trunk_layers(n)})
        self.assertEqual(zone_check.trunk_problems(b), [])

    def test_in1_has_no_slot_longer_than_10mm(self):
        """Ruling R15 on the filled placed board: no void of the In1 GND plane (the antipads round non-GND through-hole pads and holes,
        merged where a row's webs vanish, e.g. 2.54 mm socket rows) is longer than 10 mm; zone_check.in1_voids leaves out the outline's
        cutouts and the mounting / clamp holes."""
        voids = zone_check.in1_voids(self.filled())
        self.assertGreater(len(voids), 100)          # every non-GND through-hole pad leaves an antipad
        worst = max(voids)
        print(f'\nlargest In1 void {worst[0]:.2f} mm, bbox {tuple(round(x, 2) for x in worst[1])}')
        self.assertEqual([v for v in voids if v[0] > 10.0], [])

    def test_hole_keepouts_hold_only_gnd_on_every_layer(self):
        """Ruling R17b / PB3 on the filled 4-layer copy: no copper but GND (pads, tracks, vias, zone fill) on F.Cu, In1.Cu, In2.Cu or B.Cu
        within zone_check.hole_keep(h) of H1-H6 (4.5 mm round the M3 holes, CLAMP_KEEP 3.5 mm round the clamp holes). The check itself
        sees inner-layer copper: an In2 5V zone laid over H5 on another scratch copy is reported."""
        self.assertEqual(zone_check.KEEP_LAYERS, C.STACKUP)
        self.assertEqual(zone_check.hole_intrusions(self.filled()), [])
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, PRO, DRU): shutil.copy(f, Path(t) / f.name)
            b = pcbnew.LoadBoard(str(Path(t) / PCB.name))
            h5 = b.FindFootprintByReference('H5').GetPosition()
            u, v = TOMM(h5.x), TOMM(h5.y)
            z = pcbnew.ZONE(b); z.SetLayer(pcbnew.In2_Cu); z.SetNet(b.FindNet('5V'))
            o = z.Outline(); o.NewOutline()
            for du, dv in ((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)): o.Append(MM(u + du), MM(v + dv))
            z.SetAssignedPriority(50); z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER); b.Add(z)
            pcbnew.ZONE_FILLER(b).Fill([z])
            self.assertIn(('H5', 'zone', '5V', 'In2.Cu'), zone_check.hole_intrusions(b))

    def test_trunk_pins_spokes_carry_their_pad_current(self):
        """PB12 (B4 fix 1), reused by B6: every POWER_TRUNK pin - GND included - joins its zones through thermal spokes that carry the pin's
        own load (board_spec.pad_current). zone_check.pin_spoke_capacity (B6): a max-flow from the pin's spokes - per piece, the spokes of
        its pads x the zone's spoke width at that layer's copper weight (copper.ipc2221_current_a, internal for In1 / In2) - to its net's
        trunk copper, through the net's vias (each barrel copper.via_current_a) and its other soldered pins; a pocket or patch reached only
        through vias counts up to the barrels, a dead island nothing. The rows print the raw spokes per layer. board_spec.PAD_PENDING pins
        (a controller ruling pending) are the only exceptions and must still be short. The joints stay thermal reliefs (iron-solderable),
        spokes <= 2.0 mm."""
        b = self.filled()
        rows, short = [], []
        for net, pins in B.POWER_TRUNK.items():
            for ref, pin in pins:
                cap, per = zone_check.pin_spoke_capacity(b, ref, pin)
                need = B.pad_current(ref, pin)
                rows.append(f'{ref}.{pin} {net} {need:.2f} A: ' + ', '.join(f'{L} {n}x{w:g}' for L, (n, w, _) in per.items()) + f' -> {cap:.2f} A')
                if cap < need - 1e-9: short.append((ref, pin, round(cap, 2), need))
        print('\n' + '\n'.join(rows))
        for z in b.Zones():
            if not z.GetIsRuleArea(): self.assertLessEqual(TOMM(z.GetThermalReliefSpokeWidth()), 2.0, z.GetZoneName())
        self.assertEqual([s for s in short if s[:2] not in B.PAD_PENDING], [])
        self.assertEqual(sorted(s[:2] for s in short if s[:2] in B.PAD_PENDING), sorted(B.PAD_PENDING))

    def test_trunk_copper_carries_the_net_current(self):
        """PB11 (B4 fix 1), reused by B6: for every power trunk net the summed IPC-2221 capacity of its zones - trunk and helper pours alike,
        on every layer where the zone alone joins the net's POWER_TRUNK pins - is at least CURRENT_A[net]. zone_check.trunk_capacity takes
        each layer at its true narrowest copper: the largest w whose w/2 erosion of the fill still reaches one pad of every pin (contact
        level: any pad of a 4-pad fuse contact) within pad radius + relief gap + w/2, no slack (REACH_TOL is only the polygon round-off)."""
        b = self.filled()
        rows, short = [], []
        for net in (n for n in B.POWER_TRUNK if n != 'GND'):
            rep = zone_check.trunk_capacity(b, net)
            rows.append(f"{net} {B.CURRENT_A[net]} A: {rep['width_mm']} -> {rep['capacity_a']:.2f} A")
            if rep['capacity_a'] < B.CURRENT_A[net]: short.append((net, rep))
        print('\n' + '\n'.join(rows))
        self.assertEqual(short, [])

    def test_power_zones_are_generated(self):
        """B4 fix 1: POWER_ZONES and GND_PATCHES in board_spec.py are the output of pcb/gen_power_zones.py (lanes less octagon notches round
        other nets' pads and vias); re-running it would change nothing."""
        import subprocess, sys
        r = subprocess.run([sys.executable, str(D / 'gen_power_zones.py'), '--check'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, (r.stdout + r.stderr)[-2000:])

    def test_stitch_vias(self):
        """B4 fix 1: every stitching via (board_spec.STITCH_VIAS) is on the placed board with its net, through all four layers at its net
        class's via size (board_spec.net_via: Power 1.2 / 0.6, else 0.8 / 0.4 mm), locked; it lies inside its net's FILLED copper on >= 2
        layers (zone_check.via_copper_layers on the filled copy - B6 carry item 8: the polygon test this replaces counted In1 / B.Cu for
        every GND via without looking, so it could not fail), keeps >= 0.5 mm of copper gap from other nets' pads (R17a) and >= 0.4 mm
        (plus its radius) from other nets' trunk polygons, and stays >= 0.9 mm inside the board edge. A layer counts only where the via's
        fill piece is the net's trunk there or a pad of the net touches it (B6 fix 1: not merely its zone's largest piece, nor a piece
        that holds other vias). board_spec.STITCH_PENDING vias (a controller ruling pending) are the only exceptions, and must still fail.
        The check itself can fail, on a scratch copy: a GND via dropped under the VBAT_SW F.Cu strip and the F1_OUT B.Cu trunk meets GND
        copper on In1 only; and two GND vias in a one-piece F.Cu GND patch that touches no pad (its zone's largest piece, each via holding
        the other) meet live GND copper on In1 only."""
        vias = {(v.GetNetname(), round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)): v
                for v in self.b.GetTracks() if v.GetClass() == 'PCB_VIA'}
        filled = self.filled()
        inner = self.board.CloneDropTriangulation(); inner.Deflate(MM(0.9), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
        others = [(fp.GetReference(), p) for fp in self.b.GetFootprints() for p in fp.Pads()]
        def disc(c, r): return polyset([(c[0] + r * math.cos(2 * math.pi * k / 48), c[1] + r * math.sin(2 * math.pi * k / 48)) for k in range(48)])
        outline = OUTLINE['outline_uv']
        for net, (u, v) in B.STITCH_VIAS:
            via = vias.get((net, round(u, 3), round(v, 3)))
            self.assertIsNotNone(via, (net, u, v))
            self.assertTrue(via.IsLocked(), (net, u, v))
            self.assertEqual(via.GetViaType(), pcbnew.VIATYPE_THROUGH, (net, u, v))
            dia, drill = B.net_via(net)
            self.assertAlmostEqual(TOMM(via.GetWidth(pcbnew.F_Cu)), dia, 3); self.assertAlmostEqual(TOMM(via.GetDrillValue()), drill, 3)
            c = pcbnew.VECTOR2I(MM(u), MM(v))
            own = zone_check.via_copper_layers(filled, net, (u, v))
            if (net, (u, v)) in B.STITCH_PENDING:
                self.assertIn(B.STITCH_PENDING[(net, (u, v))], B.PAD_PENDING)
                self.assertLess(len(own), 2, (net, u, v, own, 'fixed: drop it from STITCH_PENDING'))
            else:
                self.assertGreaterEqual(len(own), 2, (net, u, v, own))
            d = disc((u, v), dia / 2)
            near = [f'{r}.{p.GetNumber()}' for r, p in others if p.GetNetname() != net and abs(TOMM(p.GetPosition().x) - u) < 6
                    and abs(TOMM(p.GetPosition().y) - v) < 6 and area_mm2(both(d, self.pad_poly(p, 0.5 - 0.001))) > 1e-6]
            self.assertEqual(near, [], (net, u, v))
            for z in B.POWER_ZONES:
                if z['net'] == net or not z.get('trunk', True): continue
                g = polyset(z['poly']); g.Inflate(MM(0.4), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
                self.assertLess(area_mm2(both(d, g)), 1e-6, (net, u, v, z['net'], z['layers']))
            self.assertTrue(inner.Contains(c), (net, u, v))
        with tempfile.TemporaryDirectory() as t:                 # the check fails on a broken copy (carry item 8)
            for f in (PCB, PRO, DRU): shutil.copy(f, Path(t) / f.name)
            b = pcbnew.LoadBoard(str(Path(t) / PCB.name))
            bad = (15.0, 52.0)                                    # VBAT_SW F.Cu strip over F1_OUT's B.Cu trunk: GND only on In1
            via = pcbnew.PCB_VIA(b); via.SetPosition(pcbnew.VECTOR2I(MM(bad[0]), MM(bad[1]))); via.SetViaType(pcbnew.VIATYPE_THROUGH)
            via.SetWidth(MM(B.GND_VIA[0])); via.SetDrill(MM(B.GND_VIA[1])); via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            via.SetNet(b.FindNet('GND')); b.Add(via)
            pair = [(14.6, 55.0), (15.4, 55.0)]                   # B6 fix 1: an orphan pair in a one-piece F.Cu GND patch (no pad)
            z = pcbnew.ZONE(b); z.SetLayer(pcbnew.F_Cu); z.SetNet(b.FindNet('GND')); z.SetAssignedPriority(100)
            o = z.Outline(); o.NewOutline()
            for pu, pv in ((14.0, 54.4), (16.0, 54.4), (16.0, 55.6), (14.0, 55.6)): o.Append(MM(pu), MM(pv))
            b.Add(z)
            for pu, pv in pair:
                pv_ = pcbnew.PCB_VIA(b); pv_.SetPosition(pcbnew.VECTOR2I(MM(pu), MM(pv))); pv_.SetViaType(pcbnew.VIATYPE_THROUGH)
                pv_.SetWidth(MM(B.GND_VIA[0])); pv_.SetDrill(MM(B.GND_VIA[1])); pv_.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                pv_.SetNet(b.FindNet('GND')); b.Add(pv_)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            self.assertEqual(zone_check.via_copper_layers(b, 'GND', bad), ['In1.Cu'])
            pcs = zone_check.pieces(zone_check.solid_fill(z, pcbnew.F_Cu), min_area=0.0)
            self.assertEqual(len(pcs), 1)                         # the patch is its zone's largest (only) piece and holds both vias
            self.assertTrue(all(pcs[0].Contains(pcbnew.VECTOR2I(MM(pu), MM(pv))) for pu, pv in pair))
            for uv in pair: self.assertEqual(zone_check.via_copper_layers(b, 'GND', uv), ['In1.Cu'], uv)

    def test_gnd_patches_join_their_pins(self):
        """Ruling PB21 (B6 fix 2): on the filled board every F.Cu GND patch joins its pin through real spokes and holds stitching vias that
        tie live copper (patch_problems). U3P.2 can take one spoke per outer layer only (its other sides are U3P.1's VMOT pad, U3P.3's
        Motor clearance and the L_M1 track), so board_spec.DRC_RULES accepts one spoke (min_resolved_spokes 1) for that pad alone - the
        pads of PAD_SPOKE_ANGLE, each with exactly one F.Cu and one B.Cu spoke. The check fails on B6 fix 1's copper (3a1fbdf, rebuilt on
        a scratch copy: GND_PATCH_2 back to u 46.5, U3P.2's spokes back at 45 degrees, its second via gone): the patch gives U3P.2 no
        spoke and its via meets live copper on In1 only."""
        spokes, vias, held = patch_problems(self.filled())
        self.assertEqual((spokes, vias), ([], []))
        self.assertTrue(all(held.values()), held)
        rules = re.findall(r'\(rule "[^"]+"\s*\(constraint min_resolved_spokes (\d+)\)\s*\(condition "([^"]+)"\)\)', B.DRC_RULES)
        self.assertEqual([r[0] for r in rules], ['1'])
        self.assertEqual(set(B.PAD_SPOKE_ANGLE), {('U3P', '2')})
        self.assertEqual(rules[0][1], "A.Type == 'Pad' && A.memberOfFootprint('U3P') && A.Pad_Number == '2'")     # B12a (carry 10): the whole condition
        sp = zone_check.pin_spokes(self.filled(), 'U3P', '2')
        self.assertEqual((sp['F.Cu'][0], sp['B.Cu'][0]), (1, 1))
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, PRO, DRU): shutil.copy(f, Path(t) / f.name)
            b = pcbnew.LoadBoard(str(Path(t) / PCB.name))
            z = next(z for z in b.Zones() if z.GetZoneName() == 'GND_PATCH_2')
            o = z.Outline(); o.RemoveAllContours(); o.NewOutline()
            for u, v in ((42.7, 43.65), (46.5, 43.65), (46.5, 45.2), (42.7, 45.2)): o.Append(MM(u), MM(v))
            next(p for p in b.FindFootprintByReference('U3P').Pads() if p.GetNumber() == '2').SetThermalSpokeAngleDegrees(45)
            for v in [v for v in b.GetTracks() if v.GetClass() == 'PCB_VIA' and v.GetNetname() == 'GND'
                      and (round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)) == (45.6, 44.25)]: b.Delete(v)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            spokes, vias, held = patch_problems(b)
            self.assertEqual(spokes, [('GND_PATCH_2', 'U3P.2', 0)])
            self.assertEqual(vias, [((43.4, 44.25), ['In1.Cu'])])

    @staticmethod
    def pad_poly(p, grow=0.0):
        ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, pcbnew.F_Cu, MM(grow), MM(0.005), pcbnew.ERROR_OUTSIDE); return ps

    # --- placement intent (bay B3: the board under the Jetson) ---
    DIRS = {'+u': (1, 0), '-u': (-1, 0), '+v': (0, 1), '-v': (0, -1)}

    def envelope(self, ref):
        """(u0, v0, u1, v1) of the connector's courtyard, grown in its entry direction by MATED_REACH (ruling PB6: the mated plug of a
        right-angle header lies on the board past the courtyard)."""
        u0, v0, u1, v1 = box(self.fps[ref].GetCourtyard(pcbnew.F_CrtYd))
        r = B.MATED_REACH.get(ref, 0.0)
        du, dv = self.DIRS[B.EDGE_CONNECTORS[ref]]
        return (u0 + min(du, 0) * r, v0 + min(dv, 0) * r, u1 + max(du, 0) * r, v1 + max(dv, 0) * r)

    def edge_gap(self, ref):
        """mm from the middle of the envelope's outward side to the board outline, stepping outward along the entry direction."""
        u0, v0, u1, v1 = self.envelope(ref)
        du, dv = self.DIRS[B.EDGE_CONNECTORS[ref]]
        u = {-1: u0, 1: u1, 0: (u0 + u1) / 2}[du]; v = {-1: v0, 1: v1, 0: (v0 + v1) / 2}[dv]
        for k in range(400):
            if not self.board.Contains(pcbnew.VECTOR2I(MM(u + du * k * 0.05), MM(v + dv * k * 0.05))): return k * 0.05
        return 20.0

    def test_connectors_on_their_edges(self):
        """Every off-board connector and P03 terminal (EDGE_CONNECTORS) sits within 1.5 mm of the outline edge in its entry direction: its
        courtyard, or for a right-angle header whose mated plug lies on the board the plug's far end (courtyard + MATED_REACH, PB6),
        and that plug footprint lies on the board."""
        for ref in B.EDGE_CONNECTORS:
            self.assertLessEqual(self.edge_gap(ref), 1.5, f'{ref} not on its {B.EDGE_CONNECTORS[ref]} edge')
            if ref in B.MATED_REACH:
                self.assertLess(area_mm2(minus(polyset(B.rect_pts(self.envelope(ref))), self.board)), 0.01, f'{ref}: mated plug off the board')

    def test_connector_plug_entry_faces_outward(self):
        """The entry side of every edge connector (B.ENTRY_LOCAL, in its footprint's own frame: the VH / XH latch side of a vertical header,
        the plug face of a side-entry header, the pin tips of a right-angle pin header, the MKDS wire entry), rotated by the footprint's
        orientation on the board, points along its EDGE_CONNECTORS direction."""
        for ref, d in B.EDGE_CONNECTORS.items():
            lx, ly = B.ENTRY_LOCAL[B.PARTS[ref]['fp'].split(':')[1]]
            du, dv = B.rot(self.fps[ref].GetOrientationDegrees(), lx, ly)
            self.assertEqual((round(du), round(dv)), self.DIRS[d], f'{ref}: entry side faces ({du:.1f}, {dv:.1f}), not {d}')

    def test_assembly_rules_for_reflow(self):
        """Spec 2026-10-06 section 3: every footprint on the top side (SMD on top only - F.Cu, never In1 / In2 / B.Cu - so the board can sit on a hot plate); SMD
        courtyards >= SMD_GAP (1.0 mm) apart for rework; every SMD pad >= SMD_TALL_GAP (2.0 mm) from every through-hole pad (ruling LP9:
        relay, fuse holders, C1, sockets, all connectors and terminals) except the M3 holes' own pads, so the hot air is not blocked."""
        for ref, fp in self.fps.items():
            self.assertEqual(fp.GetLayer(), pcbnew.F_Cu, f'{ref} is on the bottom side')
            if B.PARTS[ref]['mount'] == 'smd_top':
                for p in fp.Pads():
                    self.assertEqual(p.GetAttribute(), pcbnew.PAD_ATTRIB_SMD, ref); self.assertTrue(p.IsOnLayer(pcbnew.F_Cu), ref)
                    for L in ('In1.Cu', 'In2.Cu', 'B.Cu'): self.assertFalse(p.IsOnLayer(LAYER[L]), (ref, L))   # B4: top copper only
        smd = sorted(r for r in self.fps if B.PARTS[r]['mount'] == 'smd_top')
        cy = {r: self.fps[r].GetCourtyard(pcbnew.F_CrtYd) for r in smd}
        close = []
        for i, a in enumerate(smd):
            g = cy[a].CloneDropTriangulation(); g.Inflate(MM(B.SMD_GAP - 0.001), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
            close += [(a, c) for c in smd[i + 1:] if area_mm2(both(g, cy[c])) > 1e-5]
        self.assertEqual(close, [], f'SMD courtyards closer than {B.SMD_GAP} mm')
        def pad_poly(p):
            ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, pcbnew.F_Cu, 0, MM(0.005), pcbnew.ERROR_OUTSIDE); return ps
        tall = [(r, p, pad_poly(p)) for r in self.fps if B.PARTS[r]['mount'] == 'tht' and r not in B.THT_GAP_EXEMPT for p in self.fps[r].Pads()]
        self.assertGreaterEqual(len(tall), 150)          # every connector, socket, terminal, relay, fuse-holder and C1 pad
        near = []
        for a in smd:
            for p in self.fps[a].Pads():
                g = pad_poly(p); g.Inflate(MM(B.SMD_TALL_GAP - 0.001), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
                near += [(a, p.GetNumber(), r, q.GetNumber()) for r, q, qp in tall if area_mm2(both(g, qp)) > 1e-5]
        self.assertEqual(near, [], f'SMD pads closer than {B.SMD_TALL_GAP} mm to through-hole pads')

    def footprint_area(self, ref):
        """The courtyard, and for a module socket also the module body (the module stands over it)."""
        shapes = [self.fps[ref].GetCourtyard(pcbnew.F_CrtYd)]
        if fab_body(self.fps[ref]) is not None: shapes.append(fab_body(self.fps[ref]))
        return shapes

    def test_part_heights_fit_where_they_sit(self):
        """Every part's height <= B.max_part_h_at at every 0.5 mm cell of its courtyard and module body: MAX_PART_H (lid underside - 3 mm)
        behind the Jetson carrier, UNDER_CARRIER_MAX_H (16.27 mm) under it. A connector's height is the larger of its body (PARTS h) and its
        mated plug with the wire bend over a vertical part (MATED_H + CABLE_BEND_MM)."""
        bad = []
        for ref in self.fps:
            h = effective_h(ref)
            lim = B.MAX_PART_H
            for s in self.footprint_area(ref):
                u0, v0, u1, v1 = box(s)
                for i in range(int((u1 - u0) / 0.5) + 2):
                    for j in range(int((v1 - v0) / 0.5) + 2):
                        u, v = min(u0 + 0.5 * i, u1), min(v0 + 0.5 * j, v1)
                        if s.Contains(pcbnew.VECTOR2I(MM(u), MM(v))) or s.Collide(pcbnew.VECTOR2I(MM(u), MM(v)), MM(0.01)):
                            lim = min(lim, B.max_part_h_at(u, v))
            if h > lim + 1e-9: bad.append((ref, h, lim))
        self.assertEqual(bad, [])

    def test_only_smd_parts_under_modules(self):
        """Spec section 3: SMD parts may sit under the socketed modules (reflowed before the modules go in), through-hole parts may not;
        anything under a module is lower than MODULE_CLEARANCE, and module bodies do not overlap each other."""
        bodies = {name: fab_body(self.fps[next(iter(m['rows']))]) for name, m in B.MODULES.items()}
        for name, m in B.MODULES.items():
            for ref, fp in self.fps.items():
                if ref in m['rows'] or area_mm2(both(bodies[name], fp.GetCourtyard(pcbnew.F_CrtYd))) < 1e-4: continue
                self.assertEqual(B.PARTS[ref]['mount'], 'smd_top', f'{ref} under the {name}')
                self.assertLess(B.PARTS[ref]['h'], B.MODULE_CLEARANCE, ref)
        names = sorted(bodies)
        for i, a in enumerate(names):
            for c in names[i + 1:]: self.assertLess(area_mm2(both(bodies[a], bodies[c])), 1e-4, (a, c))

    def test_m3_washers_touch_only_gnd(self):
        """Ruling R17 (and PB3 for the clamp holes): no copper but GND within zone_check.hole_keep(h) of a mounting-hole centre - 4.5 mm
        round the M3 holes H1-H4, CLAMP_KEEP 3.5 mm round the clamp holes H5 / H6 - no other net's pad, and no F.Cu power-trunk polygon
        (L6 keeps tracks and vias out). Bay B4: every POWER_ZONES polygon on any layer; the filled-copy check of all four layers is
        test_hole_keepouts_hold_only_gnd_on_every_layer."""
        from pcb import zone_check
        self.assertEqual(zone_check.MOUNT_HOLES, ('H1', 'H2', 'H3', 'H4', 'H5', 'H6'))
        for h in zone_check.MOUNT_HOLES:
            c, keep = self.fps[h].GetPosition(), zone_check.hole_keep(h)
            circ = polyset([(TOMM(c.x) + keep * math.cos(2 * math.pi * k / 64), TOMM(c.y) + keep * math.sin(2 * math.pi * k / 64)) for k in range(64)])
            for ref, fp in self.fps.items():
                if ref == h: continue
                for p in fp.Pads():
                    if p.GetNetname() == 'GND': continue
                    ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, pcbnew.F_Cu, 0, MM(0.005), pcbnew.ERROR_OUTSIDE)
                    self.assertLess(area_mm2(both(circ, ps)), 1e-5, f'{ref}.{p.GetNumber()} ({p.GetNetname()}) in the {h} washer')
            for z in B.POWER_ZONES:                         # bay B4: every POWER_ZONES polygon, on every layer it is poured on
                self.assertLess(area_mm2(both(circ, polyset(z['poly']))), 1e-5, f"{z['net']} {z['layers']} zone in the {h} washer")

    def test_clamp_holes_at_outline_clamp_holes(self):
        """PB3: H5 / H6 are the rear Jetson standoff clamp holes, Ø3.0 NPTH (no copper of their own) at board_outline clamp_holes_uv
        +-0.05 mm, and no copper but GND lies within CLAMP_KEEP (3.5 mm) of them."""
        from pcb import zone_check
        self.assertEqual(zone_check.CLAMP_KEEP, 3.5)
        for i, (u, v) in enumerate(OUTLINE['clamp_holes_uv'], 5):
            fp = self.fps[f'H{i}']
            p = fp.GetPosition()
            self.assertLessEqual(abs(TOMM(p.x) - u), 0.05, f'H{i}'); self.assertLessEqual(abs(TOMM(p.y) - v), 0.05, f'H{i}')
            pads = list(fp.Pads())
            self.assertEqual(len(pads), 1, f'H{i}')
            self.assertEqual(pads[0].GetAttribute(), pcbnew.PAD_ATTRIB_NPTH, f'H{i}')
            self.assertAlmostEqual(TOMM(pads[0].GetDrillSize().x), 3.0, 3, f'H{i}')
            self.assertEqual(zone_check.hole_keep(f'H{i}'), zone_check.CLAMP_KEEP)
            c = p
            circ = polyset([(TOMM(c.x) + 3.5 * math.cos(2 * math.pi * k / 64), TOMM(c.y) + 3.5 * math.sin(2 * math.pi * k / 64)) for k in range(64)])
            for ref, other in self.fps.items():
                if ref == f'H{i}': continue
                for q in other.Pads():
                    if q.GetNetname() == 'GND': continue
                    ps = pcbnew.SHAPE_POLY_SET(); q.TransformShapeToPolygon(ps, pcbnew.F_Cu, 0, MM(0.005), pcbnew.ERROR_OUTSIDE)
                    self.assertLess(area_mm2(both(circ, ps)), 1e-5, f'{ref}.{q.GetNumber()} in the H{i} clamp keep-out')
            for z in B.POWER_ZONES:
                self.assertLess(area_mm2(both(circ, polyset(z['poly']))), 1e-5, f"{z['net']} {z['layers']} zone in the H{i} clamp keep-out")

    def test_pico_usb_at_the_left_edge_with_clear_plug_path(self):
        """Review Focus 5: the receptacle end of the Pico module (U1A / U1B) points -v, at the left edge (v 0), within 3 mm of it, and a
        USB_PLUG_W wide lane from the receptacle to v 0 holds no courtyard but the Pico's own sockets."""
        x0, y0, x1, y1 = B.MODULES['Pico']['body']
        near = B.local_to_board(B.PLACEMENT['U1A'], ((x0 + x1) / 2, y0))      # middle of the USB end of the module
        far = B.local_to_board(B.PLACEMENT['U1A'], ((x0 + x1) / 2, y1))
        self.assertAlmostEqual(near[0], far[0], 6); self.assertLess(near[1], far[1], 'USB end does not point -v')
        self.assertTrue(0.0 <= near[1] <= 3.0, f'receptacle {near[1]:.2f} mm from the left edge')
        half = B.USB_PLUG_W / 2
        path = polyset([(near[0] - half, -1.0), (near[0] + half, -1.0), (near[0] + half, near[1]), (near[0] - half, near[1])])
        for ref, fp in self.fps.items():
            if ref in B.MODULES['Pico']['rows']: continue
            self.assertLess(area_mm2(both(path, fp.GetCourtyard(pcbnew.F_CrtYd))), 1e-4, f'{ref} in the USB plug path')

    def test_band_holds_the_frequent_connectors_and_fuses(self):
        """Exposed with the lid off, behind the Jetson carrier: the battery, rocker, toggle-loop, arm-button and rear-cliff connectors, both
        fuses, the relay, C1 and the Jetson 5 V lead (PB5 fallback) have every courtyard point at u < CARRIER_U. The Pico (ruling PB7): its
        micro-USB receptacle end and the USB_PLUG_W plug path lie at u < CARRIER_U, and wherever the module reaches past CARRIER_U (its right
        socket's courtyard, 0.36 mm) it is within the under-carrier height limit (the general height test covers it too)."""
        for ref in ['J1', 'J2', 'J3', 'J6', 'J7', 'J12', 'J13', 'F1', 'F2', 'K1', 'C1']:
            for s in self.footprint_area(ref):
                self.assertLess(box(s)[2], B.CARRIER_U, f'{ref} reaches u {box(s)[2]:.2f}')
        x0, y0, x1, y1 = B.MODULES['Pico']['body']
        mid = B.local_to_board(B.PLACEMENT['U1A'], ((x0 + x1) / 2, y0))
        self.assertLess(mid[0] + B.USB_PLUG_W / 2, B.CARRIER_U, 'USB receptacle / plug path reaches the carrier zone')
        reach = max(box(s)[2] for r in ('U1A', 'U1B') for s in self.footprint_area(r))
        if reach >= B.CARRIER_U:
            self.assertLessEqual(effective_h('U1A'), B.UNDER_CARRIER_MAX_H)
            self.assertLess(reach, B.CARRIER_U + 0.4, f'Pico reaches u {reach:.2f}')     # B12a (carry 8): PB7's 0.36 mm (u 28.38), was + 0.5

    def test_under_carrier_flag_matches_the_placed_courtyards(self):
        """B12a (carry 8): board_spec.UNDER_CARRIER (its own parse of the footprint file, placed by PLACEMENT) agrees with the saved board:
        True exactly for the parts whose KiCad courtyard, as placed, reaches u >= CARRIER_U."""
        kicad = {r for r, fp in self.fps.items() if box(fp.GetCourtyard(pcbnew.F_CrtYd))[2] >= B.CARRIER_U}
        self.assertEqual(kicad, {r for r in self.fps if B.UNDER_CARRIER(r)})

    def test_dnp_spare_rows_fit_a_bare_header(self):
        """Ruling PB8: J14 / J15 are placed DNP pad rows (KiCad DNP attribute, out of the BOM); a bare 2.54 mm header fitted there later
        (BARE_HEADER_H, not a mated plug) stays within the height limit at every point of its courtyard, and they are no edge connectors."""
        for ref in ('J14', 'J15'):
            fp = self.fps[ref]
            self.assertTrue(fp.IsDNP(), ref)
            self.assertEqual(B.PARTS[ref]['h'], B.BARE_HEADER_H, ref)
            u0, v0, u1, v1 = box(fp.GetCourtyard(pcbnew.F_CrtYd))
            lim = min(B.max_part_h_at(u, v) for u in (u0, u1, (u0 + u1) / 2) for v in (v0, v1, (v0 + v1) / 2))
            self.assertLessEqual(B.BARE_HEADER_H, lim, ref)
            self.assertNotIn(ref, B.EDGE_CONNECTORS)

    def test_c1_within_30mm_of_both_vin(self):
        """Spec: a bulk capacitor within 30 mm of each driver's VIN pin (U3P.1, U4P.1), measured from the capacitor's centre (its
        courtyard's centre). Ruling PB16: the nearest of the VMOT bulk capacitors counts - C1, and C3 for DRV_L once its carrier turned."""
        caps = {}
        for ref in ('C1', 'C3'):
            u0, v0, u1, v1 = box(self.fps[ref].GetCourtyard(pcbnew.F_CrtYd))
            caps[ref] = ((u0 + u1) / 2, (v0 + v1) / 2)
        d = {}
        for ref in ('U3P', 'U4P'):
            vin = [p for p in self.fps[ref].Pads() if p.GetNumber() == str(B.DRV_PINS['VIN'][1])][0].GetPosition()
            d[ref] = min((math.hypot(cu - TOMM(vin.x), cv - TOMM(vin.y)), c) for c, (cu, cv) in caps.items())
        print(f"\nnearest bulk cap to VIN: U3P.1 {d['U3P'][0]:.1f} mm ({d['U3P'][1]}), U4P.1 {d['U4P'][0]:.1f} mm ({d['U4P'][1]})")
        for ref, (x, c) in d.items(): self.assertLessEqual(x, 30.0, (ref, c))

    def test_under_carrier_connectors_leave_a_cable_path(self):
        """Review Focus 1: every connector under the carrier keeps its mated plug (+ the wire bend over a vertical part) under
        UNDER_CARRIER_MAX_H, and nothing taller than 3 mm stands between its entry side and the board edge in its entry direction. A
        right-angle header's lay-flat plug (MATED_REACH) is part of that lane, and since the plug lies on the board (z 0..MATED_H) no
        other courtyard at all may enter its footprint (a screw head or an SMD part would lift it). The motor cables' own corridor from
        the motor exit is test_motor_cables_have_a_corridor_from_the_motor_exit."""
        under = [r for r in B.EDGE_CONNECTORS if B.UNDER_CARRIER(r)]
        self.assertTrue({'J4', 'J5', 'J8', 'J9', 'J10', 'J11', 'U5A', 'U5B'} <= set(under), under)
        for ref, d in B.EDGE_CONNECTORS.items():
            if not B.UNDER_CARRIER(ref):
                continue
            fam = B.connector_family(ref)          # 'XH' | 'VH' | 'PinHeader' | 'Terminal'
            vertical = B.PARTS[ref]['fp'].endswith('_Vertical')
            need = B.MATED_H[ref] + (B.CABLE_BEND_MM[fam] if vertical else 0.0)
            self.assertLessEqual(need, B.UNDER_CARRIER_MAX_H, ref)
            # nothing taller than 3 mm between the connector's entry side and the board edge in direction d
            self.assertEqual(cable_lane_obstacles(self.board, ref, d, max_h=3.0), [], ref)
            if ref in B.MATED_REACH:
                self.assertEqual(cable_lane_obstacles(self.board, ref, d, max_h=-1.0, plug_only=True), [], f'{ref}: something under its plug')

    def test_motor_cables_have_a_corridor_from_the_motor_exit(self):
        """Review Focus 1 for the motors' factory cables (B3 fix 1): each cable leaves its motor at the fixed exit point B.MOTOR_CABLE_EXIT
        (from the motor geometry, not from any cable placeholder) and must reach the far end of its connector's lay-flat plug through a
        clear corridor. The cable is a swept box MOTOR_CABLE_W wide (flat in plan) and MOTOR_CABLE_T thick: it runs beside the motor's inner
        end face, from the exit to the plug, over the band [face, face + W] on the board side, in u from the plug's far side to W/2 past the
        exit. No other part (courtyard or module body) with any height may stand in that corridor (only the h 0 mounting holes); the cable
        runs above its own plug (bottom >= MATED_H) and stays inside the under-carrier height limit; the plug's far end lies in the band."""
        W, T = B.MOTOR_CABLE_W, B.MOTOR_CABLE_T
        for ref in ('J4', 'J5'):
            eu, ev, ez = B.MOTOR_CABLE_EXIT[ref]
            d = B.EDGE_CONNECTORS[ref]
            self.assertIn(d, ('-v', '+v'), ref)
            v0, v1 = (ev, ev + W) if d == '-v' else (ev - W, ev)               # the board side of the motor's end face
            pu0, pv0, pu1, pv1 = self.envelope(ref)                              # courtyard + lay-flat plug
            far = pv0 if d == '-v' else pv1
            self.assertTrue(v0 <= far <= v1, f'{ref}: plug far end v {far:.2f} outside the cable band {v0:.1f}..{v1:.1f}')
            cu0, cu1 = min(pu0, eu - W / 2), max(pu1, eu + W / 2)
            corridor = polyset(B.rect_pts((cu0, v0, cu1, v1)))
            hit = []
            for other, fp in self.fps.items():
                if other == ref or effective_h(other) <= 0.0: continue
                shapes = [fp.GetCourtyard(pcbnew.F_CrtYd)] + ([fab_body(fp)] if fab_body(fp) is not None else [])
                if any(area_mm2(both(corridor, sh)) > 1e-4 for sh in shapes): hit.append(other)
            self.assertEqual(sorted(hit), [], f'{ref}: parts in the motor-cable corridor u {cu0:.1f}..{cu1:.1f}, v {v0:.1f}..{v1:.1f}')
            self.assertGreaterEqual(ez - T / 2, B.MATED_H[ref], f'{ref}: the cable would run under its own plug')
            self.assertLessEqual(ez + T / 2, B.UNDER_CARRIER_MAX_H, ref)

    def test_smd_gnd_vias_follow_lp12(self):
        """Ruling LP12 (regenerated for the bay placement): one tented GND via per top-side SMD GND pad (board_spec.SMD_GND_VIAS), at the
        SMD-pad via size (board_spec.SMD_PAD_VIA, 0.8 / 0.3 mm since LIO-226), 0.3 mm past one end of its pad so its copper overlaps that pad by about 0.1 mm while its drill stays
        off the pad (outside the paste opening); R17a: >= 0.5 mm of copper gap to every other net's pad; it keeps off the power-trunk
        polygons on every layer (>= 0.4 mm, so the corridors stay whole; a through via pierces all four layers; helper pours with
        trunk=False may hold vias) and stays >= 0.5 mm inside the board edge."""
        smd_gnd = {fp.GetReference(): [p for p in fp.Pads() if p.GetNetname() == 'GND' and not p.IsOnLayer(pcbnew.B_Cu)] for fp in self.b.GetFootprints()}
        smd_gnd = {r: ps for r, ps in smd_gnd.items() if ps}
        self.assertEqual(set(B.SMD_GND_VIAS), set(smd_gnd))
        zones = [polyset(z['poly']) for z in B.POWER_ZONES if z.get('trunk', True)]
        inner = self.board.CloneDropTriangulation(); inner.Deflate(MM(0.5 + 0.4), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
        def disc(c, r): return polyset([(c[0] + r * math.cos(2 * math.pi * k / 48), c[1] + r * math.sin(2 * math.pi * k / 48)) for k in range(48)])
        def pad_poly(p, grow=0.0):
            ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, pcbnew.F_Cu, MM(grow), MM(0.005), pcbnew.ERROR_OUTSIDE); return ps
        others = [(fp.GetReference(), p) for fp in self.b.GetFootprints() for p in fp.Pads() if p.GetNetname() != 'GND']
        for ref, (u, v) in B.SMD_GND_VIAS.items():
            via, drill = disc((u, v), B.SMD_PAD_VIA[0] / 2), disc((u, v), B.SMD_PAD_VIA[1] / 2)
            overlap = max(area_mm2(both(via, pad_poly(p))) for p in smd_gnd[ref])
            self.assertGreater(overlap, 0.005, f'{ref}: via does not touch its GND pad')
            for p in smd_gnd[ref]: self.assertLess(area_mm2(both(drill, pad_poly(p))), 1e-4, f'{ref}: via drill in the pad')
            near = [f'{r}.{p.GetNumber()}' for r, p in others if abs(TOMM(p.GetPosition().x) - u) < 6 and abs(TOMM(p.GetPosition().y) - v) < 6
                    and area_mm2(both(via, pad_poly(p, 0.5 - 0.001))) > 1e-6]
            self.assertEqual(near, [], f'{ref}: via within 0.5 mm of another net\'s pad (R17a)')
            for z in zones:
                g = z.CloneDropTriangulation(); g.Inflate(MM(0.4), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
                self.assertLess(area_mm2(both(via, g)), 1e-6, f'{ref}: via in a power-trunk zone')
            self.assertTrue(inner.Contains(pcbnew.VECTOR2I(MM(u), MM(v))), f'{ref}: via too near the board edge')
        pts = sorted(B.SMD_GND_VIAS.values())
        for i, a in enumerate(pts):
            for b_ in pts[i + 1:]: self.assertGreaterEqual(math.dist(a, b_), 0.8 + 0.25, (a, b_))

    # --- spec 2026-10-06 section 3: values and polarity marks on the silkscreen (L5 fix 1) ---
    def silk_texts(self, fp):
        return [it for it in fp.GraphicalItems() if it.GetClass() == 'PCB_TEXT' and it.GetLayer() == pcbnew.F_SilkS and it.IsVisible()
                and it.GetText() not in B.PIN_MARKS.get(fp.GetReference(), {}).values()]       # LIO-242: not the 'VIN' pin marks

    def test_every_part_has_its_value_label_on_silk(self):
        """Every part but the M3 holes carries exactly one F.SilkS label "<ref> <value or function>" (board_spec.silk_label), 0.8 mm high
        with a 0.15 mm stroke; for every SMD part it shows the reference and the value. The reference / value fields stay off the silk."""
        for ref, fp in self.fps.items():
            texts = self.silk_texts(fp)
            self.assertFalse(fp.Reference().GetLayer() == pcbnew.F_SilkS and fp.Reference().IsVisible(), ref)
            self.assertFalse(fp.Value().GetLayer() == pcbnew.F_SilkS and fp.Value().IsVisible(), ref)
            if B.silk_label(ref) is None: self.assertEqual(texts, [], ref); continue
            self.assertEqual([x.GetShownText(False) for x in texts], [B.silk_label(ref)], ref)
            x = texts[0]
            self.assertAlmostEqual(TOMM(x.GetTextHeight()), B.SILK_TEXT_H, 3, ref); self.assertAlmostEqual(TOMM(x.GetTextThickness()), B.SILK_STROKE, 3, ref)
            self.assertTrue(0.8 <= B.SILK_TEXT_H and 0.12 <= B.SILK_STROKE <= 0.15)
            if B.PARTS[ref]['mount'] == 'smd_top':
                shown = x.GetShownText(False).split()
                self.assertEqual((shown[0], ' '.join(shown[1:])), (ref, fp.GetValue()), ref)

    def test_silk_labels_sit_next_to_their_part(self):
        """Each label is nearer its own part's courtyard than any other part's (so R1 cannot be read as R2), off every part body, and on
        the board."""
        from pcb.build_pcb import box_mm, box_gap
        bodies = {r: box_mm(fp.GetCourtyard(pcbnew.F_CrtYd).BBox()) for r, fp in self.fps.items()}
        for ref, fp in self.fps.items():
            for x in self.silk_texts(fp):
                lb = box_mm(x.GetEffectiveTextShape().BBox())
                own = box_gap(lb, bodies[ref])
                near = min((box_gap(lb, b), r) for r, b in bodies.items() if r != ref)
                self.assertLess(own, near[0], f'{ref} label nearer {near[1]}')
                self.assertLess(own, 6.0, ref)
                for r, b in bodies.items():
                    if r != ref: self.assertGreater(box_gap(lb, b), 0.0, f'{ref} label over {r}')
                self.assertLess(area_mm2(minus(polyset([(lb[0], lb[1]), (lb[2], lb[1]), (lb[2], lb[3]), (lb[0], lb[3])]), self.board)), 1e-3, ref)

    def test_polarity_marks_on_silk(self):
        """Polarised parts carry their polarity on the silk (stock footprint marks, asserted here): LEDs and D1 a cathode bar beyond pad 1
        and nothing beyond pad 2; the SOT-23s a filled pin-1 mark nearest pad 1; C1 a "+" (two short crossing strokes) nearer pad 1."""
        self.assertEqual(set(B.POLARISED), {'LED1', 'LED2', 'LED3', 'D1', 'D2', 'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'C1', 'C3'})   # C3: PB16, D2: LIO-226
        def pt(p): return (TOMM(p.x), TOMM(p.y))
        for ref in B.POLARISED:
            fp = self.fps[ref]
            pads = {p.GetNumber(): p for p in fp.Pads()}
            shapes = [it for it in fp.GraphicalItems() if it.GetClass() == 'PCB_SHAPE' and it.GetLayer() == pcbnew.F_SilkS]
            p1 = pt(pads['1'].GetPosition())
            if ref.startswith(('LED', 'D')):
                p2 = pt(pads['2'].GetPosition()); L = math.dist(p1, p2); ax = ((p2[0] - p1[0]) / L, (p2[1] - p1[1]) / L)
                half = max(TOMM(pads['1'].GetSize().x), TOMM(pads['1'].GetSize().y)) / 2
                proj = [((q[0] - p1[0]) * ax[0] + (q[1] - p1[1]) * ax[1]) for s in shapes if s.GetShape() == pcbnew.SHAPE_T_SEGMENT
                        for q in (pt(s.GetStart()), pt(s.GetEnd()))]
                self.assertLess(min(proj), -half * 0.9, f'{ref}: no cathode bar beyond pad 1')
                self.assertLess(max(proj), L + half, f'{ref}: silk beyond pad 2 too (mark not one-sided)')
            elif ref.startswith('Q'):
                polys = [s for s in shapes if s.GetShape() == pcbnew.SHAPE_T_POLY]
                self.assertTrue(polys, f'{ref}: no pin-1 mark')
                c = pt(polys[0].GetCenter())
                d = {n: math.dist(c, pt(p.GetPosition())) for n, p in pads.items()}
                self.assertEqual(min(d, key=d.get), '1', f'{ref}: pin-1 mark nearest pad {min(d, key=d.get)}')
            else:                                                         # C1, C3 (PB16)
                p2 = pt(pads['2'].GetPosition())
                segs = [s for s in shapes if s.GetShape() == pcbnew.SHAPE_T_SEGMENT and TOMM(s.GetLength()) <= 1.2]
                plus = [(a, b) for a in segs for b in segs if a is not b and a.GetStart().x != a.GetEnd().x and b.GetStart().y != b.GetEnd().y
                        and math.dist(pt(a.GetCenter()), pt(b.GetCenter())) < 0.2]
                self.assertTrue(plus, f'{ref}: no + mark')
                m = pt(plus[0][0].GetCenter())
                self.assertLess(math.dist(m, p1), math.dist(m, p2), f'{ref}: + mark not at the + pad')
        # Ruling PB20: connector pin marks (board_spec.PIN_MARKS, build_pcb): a '+' (two crossing PIN_MARK_LEN strokes) at a supply pin
        # and a '-' (one stroke) at a GND pin - J1's pinout changed, so its marks follow board_spec.J1_PINOUT; B12a (final review N2) adds
        # a '+' at pin 1 of the P03 terminals and of J3. B6 fix 1: they must stay visible once the part is soldered and mated - off its
        # F.Fab housing (the bounding box of every F.Fab line of the footprint) and off every other part's body: each of its F.Fab
        # drawings, its pads and, for J4 / J5, the C18 pigtail's right-angle socket on the pins (the pin row +- 7.62 mm, out to the plug
        # zone's far end; B12a: U5A.1's mark sits in J4's courtyard margin, beside the end of J4's pin row) - and each must read as its
        # pin's: no pad on the whole board nearer than that pin. LIO-242: the 'VIN' texts at U3P.1 / U4P.1 (VMOT) keep the same rules.
        from pcb.build_pcb import boxes_hit, box_mm
        self.assertEqual(set(B.PIN_MARKS), {'J1', 'U5A', 'U5B', 'J3', 'U3P', 'U4P'})
        net = {p: n for n, pins in B.NETS.items() for p in pins}
        all_pads = [(fp.GetReference(), p.GetNumber(), pt(p.GetPosition())) for fp in self.fps.values() for p in fp.Pads()]
        def bbox(pts, grow=0.0): return (min(q[0] for q in pts) - grow, min(q[1] for q in pts) - grow, max(q[0] for q in pts) + grow, max(q[1] for q in pts) + grow)
        def bb(it): r = it.GetBoundingBox(); return (TOMM(r.GetX()), TOMM(r.GetY()), TOMM(r.GetRight()), TOMM(r.GetBottom()))
        def bodies(r2):
            f2 = self.fps[r2]
            out = [bb(it) for it in f2.GraphicalItems() if it.GetClass() == 'PCB_SHAPE' and it.GetLayer() == pcbnew.F_Fab] + [bb(q) for q in f2.Pads()]
            if r2 in B.MATED_REACH:                                   # the C18 socket on J4 / J5's pins
                pu = [pt(q.GetPosition()) for q in f2.Pads()]; u0, v0, u1, v1 = self.envelope(r2)
                if max(q[0] for q in pu) - min(q[0] for q in pu) > 1.0: c = sum(q[0] for q in pu) / len(pu); out.append((c - 7.62, v0, c + 7.62, v1))
                else: c = sum(q[1] for q in pu) / len(pu); out.append((u0, c - 7.62, u1, c + 7.62))
            return out
        others = {r2: bodies(r2) for r2 in self.fps}
        for ref, marks in B.PIN_MARKS.items():
            fp = self.fps[ref]
            pads = {p.GetNumber(): pt(p.GetPosition()) for p in fp.Pads() if p.GetNumber()}      # not J3's unnumbered NPTH peg
            segs = [s for s in fp.GraphicalItems() if s.GetClass() == 'PCB_SHAPE' and s.GetLayer() == pcbnew.F_SilkS
                    and s.GetShape() == pcbnew.SHAPE_T_SEGMENT and abs(TOMM(s.GetWidth()) - B.SILK_STROKE) < 1e-6
                    and abs(TOMM(s.GetLength()) - B.PIN_MARK_LEN) < 1e-3]
            fab = [s for s in fp.GraphicalItems() if s.GetClass() == 'PCB_SHAPE' and s.GetLayer() == pcbnew.F_Fab]
            self.assertTrue(fab, ref)
            bbs = [s.GetBoundingBox() for s in fab]          # B12a: one box round every F.Fab shape (the XT30's housing is drawn in lines)
            body = [(min(TOMM(r.GetX()) for r in bbs), min(TOMM(r.GetY()) for r in bbs), max(TOMM(r.GetRight()) for r in bbs),
                     max(TOMM(r.GetBottom()) for r in bbs))]
            count, strokes = {}, {}
            for s in segs:
                c = tuple(round(x, 2) for x in pt(s.GetCenter())); count[c] = count.get(c, 0) + 1
                strokes.setdefault(c, []).append(bbox([pt(s.GetStart()), pt(s.GetEnd())], B.SILK_STROKE / 2))
            signs = {c: '+' if n == 2 else '-' for c, n in count.items()}
            for t in fp.GraphicalItems():                                 # LIO-242: text marks, boxed by their glyphs
                if t.GetClass() == 'PCB_TEXT' and t.GetLayer() == pcbnew.F_SilkS and t.GetText() in marks.values():
                    c = tuple(round(x, 2) for x in pt(t.GetPosition())); signs[c] = t.GetText()
                    strokes[c] = [box_mm(t.GetEffectiveTextShape().BBox())]
            self.assertEqual(sorted(signs.values()), sorted(marks.values()), ref)
            for c, sign in signs.items():
                near = min(pads, key=lambda n: math.dist(c, pads[n]))
                self.assertEqual(marks[near], sign, (ref, c, sign, near))
                other = next(n for n in pads if n != near)
                if sign == 'VIN': self.assertEqual(net[(ref, near)], 'VMOT', ref)
                else: self.assertEqual((net[(ref, near)] == 'GND', net[(ref, other)] == 'GND'), (sign == '-', sign == '+'), (ref, near))
                self.assertEqual(min(all_pads, key=lambda a: math.dist(c, a[2]))[:2], (ref, near), (ref, c, 'nearer another pad'))
                for sb in strokes[c]:
                    self.assertFalse(any(boxes_hit(sb, bb) for bb in body), (ref, c, sign, 'under the housing'))
                    for r2, bs in others.items():
                        if r2 != ref: self.assertFalse(any(boxes_hit(sb, x) for x in bs), (ref, c, 'under', r2))

    def test_drc_reports_no_silk_problems(self):
        """kicad-cli DRC on a temp copy: no silk_overlap and no silk_edge_clearance (labels clear of each other, of the footprint silk and
        of the board edge)."""
        import subprocess
        from pcb import tools
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, PRO, D / 'r1_power_io.kicad_dru'): shutil.copy(f, Path(t) / f.name)
            out = Path(t) / 'drc.json'
            subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--units', 'mm', '-o', str(out), str(Path(t) / PCB.name)],
                           check=True, capture_output=True)
            drc = json.loads(out.read_text(encoding='utf-8'))
        silk = [(v['type'], [i['description'] for i in v['items']]) for v in drc['violations'] if v['type'].startswith('silk')]
        self.assertEqual(silk, [])

    def test_schematic_parity_is_clean(self):
        """B7 (carry item 2): kicad-cli DRC with --schematic-parity reports no parity item and no violation of any severity. The 277
        items at cf4d589 were the schematic's local-label net names ('/VMOT'; 184), the no-connect pins' nets (15), the footprints saved
        without a library nickname (73) and the test pads' BOM flag (5)."""
        import subprocess
        from pcb import tools
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, PRO, DRU, D / 'r1_power_io.kicad_sch', D / 'sym-lib-table', D / 'fp-lib-table'): shutil.copy(f, Path(t) / f.name)
            out = Path(t) / 'drc.json'
            subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--schematic-parity', '--severity-all', '--format', 'json', '--units', 'mm',
                            '-o', str(out), str(Path(t) / PCB.name)], check=True, capture_output=True)
            drc = json.loads(out.read_text(encoding='utf-8'))
        self.assertEqual([(v['type'], v['description']) for v in drc['schematic_parity']][:10], [])
        self.assertEqual([(v['type'], v['description']) for v in drc['violations']][:10], [])
        self.assertEqual(drc['unconnected_items'][:5], [])
        self.assertTrue(all(str(fp.GetFPID().GetUniStringLibId()) == B.PARTS[fp.GetReference()]['fp'] for fp in self.b.GetFootprints()))

    # build_pcb changes these 13 library footprints on purpose (module outlines on F.Fab, the PIN_MARKS on J1 / J3 / U5A / U5B / U3P / U4P, J1's
    # stock +/- texts moved to F.Fab, inherited zone connections, U3P.2's spoke angle): the footprints lib_footprint_mismatch flags. make_schematic.DRC_SEVERITY ignores the check project-wide
    # (kicad-cli 9.0.9 does not apply per-item DRC exclusions), so this test runs it on and pins the flagged set.
    LIB_MISMATCH_REFS = {'F1', 'F2', 'J1', 'J3', 'U1A', 'U1B', 'U2', 'U3L', 'U3P', 'U4L', 'U4P', 'U5A', 'U5B'}   # B12a: J3 / U5A / U5B '+' marks

    def test_lib_footprint_mismatch_is_exactly_the_modified_footprints(self):
        """B7 fix 1 (ruling PB23, item 7): kicad-cli DRC on a temp copy with lib_footprint_mismatch back at its default (warning) flags
        exactly F1, F2, J1, J3, U1A, U1B, U2, U3L, U3P, U4L, U4P, U5A and U5B (LIB_MISMATCH_REFS, 13) - one report each - and nothing else; the committed project keeps the check
        ignored. A footprint that drifts from its library (or a new deliberate change) is a new mismatch and fails here."""
        import subprocess
        from pcb import tools
        self.assertEqual(json.loads(PRO.read_text(encoding='utf-8'))['board']['design_settings']['rule_severities'],      # LIO-238: the
                         {'lib_footprint_mismatch': 'ignore'})     # project fp-lib-table resolves the libraries, lib_footprint_issues is on
        with tempfile.TemporaryDirectory() as t:
            for f in (PCB, DRU, D / 'r1_power_io.kicad_sch', D / 'sym-lib-table', D / 'fp-lib-table'): shutil.copy(f, Path(t) / f.name)
            pro = json.loads(PRO.read_text(encoding='utf-8'))
            pro['board']['design_settings']['rule_severities'] = {}                       # every check at KiCad's default
            (Path(t) / PRO.name).write_bytes(json.dumps(pro, indent=2).encode())
            out = Path(t) / 'drc.json'
            subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--severity-all', '--format', 'json', '--units', 'mm', '-o', str(out),
                            str(Path(t) / PCB.name)], check=True, capture_output=True)
            drc = json.loads(out.read_text(encoding='utf-8'))
        flagged = [v['items'][0]['description'].removeprefix('Footprint ') for v in drc['violations'] if v['type'] == 'lib_footprint_mismatch']
        self.assertEqual(sorted(flagged), sorted(self.LIB_MISMATCH_REFS), 'a footprint differs from its library that build_pcb does not mean to change')
        self.assertEqual([v['type'] for v in drc['violations'] if v['type'] != 'lib_footprint_mismatch'], [])


if __name__ == '__main__':
    unittest.main()
