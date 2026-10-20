"""Routed-board acceptance (Task 6; bay B5: 4 layers, In1 the unrouted GND plane), run on the committed routed + zone-filled board.
Run with KiCad's Python:
    cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_route -v
DRC output goes to a temp dir (never into the repo)."""
import json, math, re, shutil, subprocess, tempfile, unittest
from collections import Counter
from pathlib import Path
import pcbnew
from pcb import board_spec as B, copper as C, route, tools, zone_check
from pcb.gridroute import OUTER_CLASSES
D = Path(B.__file__).parent
PCB, PRO = D / 'r1_power_io.kicad_pcb', D / 'r1_power_io.kicad_pro'
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
# Ruling R15 / R17 limits (the pad pairs and ratio limits for the return paths are zone_check.POWER_LOOPS / SIGNAL_RETURNS, measured on
# the In1 GND plane, copper.GND_PLANE_LAYER; the In1 slot limit is IN1_SLOT_MAX). Via sizes per net class: board_spec.CLASS_VIA.
VIA_PAD_MIN = route.VIA_PAD_KEEP   # mm: via copper to any other net's pad copper (R17a: solder side, no bridges)
IN1_SLOT_MAX = 10.0           # mm: R15, no void of the In1 GND plane longer than this (zone_check.in1_voids, measured along its bends)


def polyset(pts):
    ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
    for u, v in pts: ps.Append(MM(u), MM(v))
    return ps


def item_poly(item, layer):
    ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, layer, 0, MM(0.005), pcbnew.ERROR_OUTSIDE)
    return ps


def copy_board(t):
    for f in (PCB, PRO): shutil.copy(f, Path(t) / f.name)
    return pcbnew.LoadBoard(str(Path(t) / PCB.name))


class RouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = pcbnew.LoadBoard(str(PCB))
        cls.tracks = [t for t in cls.b.GetTracks() if t.GetClass() in ('PCB_TRACK', 'PCB_ARC')]
        cls.vias = [t for t in cls.b.GetTracks() if t.GetClass() == 'PCB_VIA']
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / 'drc.json'
            subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--severity-all', '--format', 'json', '--units', 'mm', '-o', str(out), str(PCB)],
                           check=True, capture_output=True)
            cls.drc = json.loads(out.read_text(encoding='utf-8'))

    # --- acceptance 1: DRC (no violation of any severity, as route.evaluate requires since LIO-233; no unconnected item) ---
    def test_drc_no_errors(self):
        found = [(v['severity'], v['type'], v['description'], [i['description'] for i in v['items']]) for v in self.drc['violations']]
        print('\nDRC violations by severity and category:', dict(Counter((v['severity'], v['type']) for v in self.drc['violations'])))
        self.assertEqual(found[:20], [], f'{len(found)} DRC violations')

    def test_drc_fully_connected(self):
        unc = [[i['description'] for i in u['items']] for u in self.drc['unconnected_items']]
        self.assertEqual(unc[:20], [], f'{len(unc)} unconnected items')

    # --- zones: filled, and the saved fill is current ---
    def test_zones_filled_and_current(self):
        """Every zone filled on each of its layers, and a fresh fill gives the same areas. PB18: keyed per zone and layer (zone name and
        layer, the zones in file order) - a net may own several zones on one layer (GND's two F.Cu patches), and KiCad's GetLayer() is
        the first layer of the set, not the zone's."""
        def areas(board):
            return [((z.GetNetname(), z.GetZoneName(), board.GetLayerName(L)), zone_check.area_mm2(z.GetFilledPolysList(L)))
                    for z in board.Zones() if not z.GetIsRuleArea() for L in z.GetLayerSet().Seq()]
        saved = areas(self.b)
        for key, a in saved: self.assertGreater(a, 1.0, f'{key} not filled')
        with tempfile.TemporaryDirectory() as t:
            b = copy_board(t)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            fresh = areas(b)
        self.assertEqual([k for k, _ in fresh], [k for k, _ in saved])
        for (key, a), (_, s) in zip(fresh, saved): self.assertAlmostEqual(a, s, delta=0.5, msg=f'{key} fill is stale')

    # --- acceptance 2: R4 / R13 power trunks on the routed, filled board ---
    def test_power_trunks_survive_routing(self):
        rep = zone_check.trunk_report(self.b)
        for net, r in rep.items():
            if not r['pads']: continue
            worst = max(r['pads'], key=lambda p: p[3] - p[4])
            print(f'\n{net}: {r["pieces"]} eroded piece(s), main {r["main_area"]:.0f} mm2, tightest pad {worst}', end='')
        self.assertEqual([p for r in rep.values() for p in r['problems']], [])

    # --- acceptance 3: GND plane integrity ---
    def test_every_gnd_pad_on_the_main_gnd_piece(self):
        """Every GND pad is joined to the largest piece of the In1 GND plane (copper.GND_PLANE_LAYER) through GND copper; the plane's
        largest piece is reported against the placement baseline (only through vias cut it: In1 is not routed)."""
        self.assertEqual(C.GND_PLANE_LAYER, 'In1.Cu')
        _, after = zone_check.gnd_main_piece(self.b)
        _, before = zone_check.gnd_main_piece(self.baseline())
        print(f'\nGND plane largest piece: {before:.0f} mm2 baseline -> {after:.0f} mm2 routed ({after / max(before, 1e-9):.1%})')
        self.assertEqual(zone_check.gnd_pads_off_main_piece(self.b), [])

    def test_no_gnd_tracks(self):
        """Ruling (c): GND pads join through the In1 plane (bay B5; the B.Cu GND fill is extra copper), no GND tracks. L3b made 22 GND pads top-side SMD (F.Cu only), which reach
        the plane only through a via: so zero GND tracks, and GND vias exactly the scripted board_spec.SMD_GND_VIAS, each overlapping
        one top-only GND pad (one per such pad, locked), plus the scripted GND stitching vias (B4 fix 1: GND pad patches / B.Cu islands
        to the In1 plane, board_spec.STITCH_VIAS)."""
        gnd = [(t.GetClass(), t.GetLayerName()) for t in self.tracks if t.GetNetname() == 'GND']
        self.assertEqual(gnd, [])
        self.assertEqual(zone_check.gnd_copper_problems(self.b), [])
        vias = sorted((round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3), v.IsLocked()) for v in self.vias
                      if v.GetNetname() == 'GND')
        self.assertEqual(vias, sorted([(float(u), float(v), True) for u, v in B.SMD_GND_VIAS.values()]
                                      + [(float(u), float(v), True) for n, (u, v) in B.STITCH_VIAS if n == 'GND']))
        smd = {(fp.GetReference(), p.GetNumber()) for fp in self.b.GetFootprints() for p in fp.Pads()
               if p.GetNetname() == 'GND' and not p.IsOnLayer(pcbnew.B_Cu)}
        self.assertEqual({ref for ref, _ in smd}, set(B.SMD_GND_VIAS))
        lp12 = {(round(u, 3), round(v, 3)) for u, v in B.SMD_GND_VIAS.values()}       # LIO-226: the LP12 vias are 0.8 / 0.3 on the board
        self.assertEqual({(round(TOMM(v.GetWidth(pcbnew.F_Cu)), 3), round(TOMM(v.GetDrillValue()), 3)) for v in self.vias if v.GetNetname() == 'GND'
                          and (round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)) in lp12}, {B.SMD_PAD_VIA})

    _baseline = None
    @classmethod
    def baseline(cls):
        """The placement baseline: this board with only its locked copper (the pre-routes), zones refilled."""
        if cls._baseline is None:
            cls._tmp = tempfile.TemporaryDirectory(); cls.addClassCleanup(cls._tmp.cleanup)
            b = copy_board(cls._tmp.name)
            for item in list(b.GetTracks()):
                if not item.IsLocked(): b.Delete(item)          # Delete, not Remove (see route.strip)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            cls._baseline = b
        return cls._baseline

    def test_return_paths_through_the_gnd_plane(self):
        """R15 on In1 (bay B5): through the In1 GND copper, power loops within 5 %, motor / encoder / IMU returns within 20 % of the
        placement baseline (the board with only its locked pre-routes and pre-route vias)."""
        rep = zone_check.return_paths(self.b, self.baseline(), layer=C.GND_PLANE_LAYER)
        print()
        for (a, b), base, got, ratio, limit in rep:
            print(f'  return {a[0]}.{a[1]} - {b[0]}.{b[1]}: {base:.1f} -> {got:.1f} mm ({ratio:.2f}x, max {limit})')
        self.assertEqual([(pair, round(base, 1), round(got, 1), round(ratio, 3)) for pair, base, got, ratio, limit in rep
                          if not ratio <= limit], [])

    # --- acceptance 4: widths and vias meet the net classes ---
    def test_track_and_via_sizes_meet_net_classes(self):
        width = {n: c['width'] for n, c in B.NETCLASSES.items()}
        ok = B.narrow_track_keys()                       # PB18: the listed scripted probe / lamp feeds only
        r3 = lambda p: (round(TOMM(p.x), 3), round(TOMM(p.y), 3))
        bad = []
        for t in self.tracks:
            cls = t.GetNet().GetNetClassName()
            need = width.get(cls, B.DEFAULT_CLASS['width'])
            key = (t.GetNetname(), t.GetLayerName(), round(TOMM(t.GetWidth()), 3), frozenset({r3(t.GetStart()), r3(t.GetEnd())}))
            if TOMM(t.GetWidth()) < need - 1e-6 and key not in ok: bad.append((t.GetNetname(), cls, TOMM(t.GetWidth())))
        for v in self.vias:
            cls = v.GetNet().GetNetClassName()
            at = (TOMM(v.GetPosition().x), TOMM(v.GetPosition().y))
            dia, drill = B.net_via(v.GetNetname(), at)       # PB17: the class's via, GND's pinned (LIO-226: LP12 0.8 / 0.3, stitching 0.8 / 0.4)
            if TOMM(v.GetWidth(pcbnew.F_Cu)) < dia - 1e-6 or TOMM(v.GetDrillValue()) < drill - 1e-6:
                bad.append(('via', v.GetNetname(), cls, TOMM(v.GetWidth(pcbnew.F_Cu)), TOMM(v.GetDrillValue())))
        self.assertEqual(bad, [])
        self.assertGreater(len(self.tracks), 0)

    # --- bay B5: In1 is the plane (not routed); the battery-path nets are zone copper only ---
    def test_outer_classes_stay_on_outer_layers(self):
        """R4 (and PB18 fix 1 for stage F): no router copper of a Motor- or Power-class net on In1 / In2 - their widths are sized for 1 oz
        outer copper. Only locked (scripted) copper could be there, and the pre-routes put none."""
        outer = {n for c in OUTER_CLASSES for n in B.NETCLASSES[c]['nets']}
        bad = [(t.GetNetname(), t.GetLayerName()) for t in self.tracks
               if t.GetNetname() in outer and t.GetLayer() in (pcbnew.In1_Cu, pcbnew.In2_Cu) and not t.IsLocked()]
        self.assertEqual(bad, [])
        self.assertEqual([r for r in B.PREROUTES if r['net'] in outer and r['layer'] in ('In1.Cu', 'In2.Cu')], [])

    def test_routed_copper_layers(self):
        """No track on In1.Cu (the GND plane is not routed). No router track on a Power-class net (BAT+, F1_OUT, F2_OUT): the R4 comment's
        3 mm stubs carry 10.6 A only as a pair on both outer layers, so they go on both or not at all - the bay board needs none, every
        pad of those nets is zone copper. Reports the track share per routed layer and the via count."""
        self.assertGreater(len(self.tracks), 0, 'no tracks')
        by = Counter()
        for t in self.tracks: by[t.GetLayerName()] += TOMM(t.GetLength())
        total = sum(by.values())
        print('\ntracks: ' + ', '.join(f'{L} {by[L]:.0f} mm ({by[L] / total:.1%})' for L in C.STACKUP) + f'; {total:.0f} mm in all; '
              f'vias: {len(self.vias)} ({sum(1 for v in self.vias if not v.IsLocked())} router); routed with: '
              f'{ {str(k): v for k, v in dict(self.b.GetProperties()).items()}.get("ROUTED_WITH")}')
        self.assertEqual([t for t in self.tracks if t.GetLayer() == pcbnew.In1_Cu], [])
        power = set(B.NETCLASSES['Power']['nets'])
        self.assertEqual(sorted({(t.GetNetname(), t.GetLayerName()) for t in self.tracks if t.GetNetname() in power and not t.IsLocked()}), [])

    def test_routed_with_records_the_route_defaults(self):
        """B5 fix 1 / B6 fix 1 (review minor 6): route.py with no arguments reproduces the committed board, so the board's ROUTED_WITH
        record must name route.py's defaults exactly - DEFAULT_GRID (and its SETUPS name), DEFAULT_ORDER, DEFAULT_PASSES and
        CRITICAL_NETS (the Motor class + DRIVER_BUNDLE). A default changed without a re-route, or a board routed with other arguments,
        fails here. LIO-228: route.py also records the tools (KiCad version, Freerouting jar sha256, Java version): KiCad 9.0.x, the
        pinned jar (tools.FREEROUTING_SHA256) and Java 25, required (LIO-233: the committed board carries them)."""
        from pcb import route
        rec = {str(k): v for k, v in dict(self.b.GetProperties()).items()}.get('ROUTED_WITH', '')
        m = re.fullmatch(r'route\.py set-up (\d+) (\S+) wiring (\S+) stage ([A-F]) passes (\d+) critical (\S+)'
                         r' kicad (\S+) freerouting-sha256 ([0-9a-f]{64}) java (\S+)', rec)
        self.assertIsNotNone(m, rec)
        g, name, order, _stage, passes, critical, kicad, jar, java = m.groups()
        self.assertEqual((kicad[:4], jar, java.split('.')[0]), ('9.0.', tools.FREEROUTING_SHA256, '25'), rec)
        self.assertEqual((int(g), name, order, int(passes), critical.split(',')),
                         (route.DEFAULT_GRID, route.SETUPS[route.DEFAULT_GRID]['name'], route.DEFAULT_ORDER, route.DEFAULT_PASSES,
                          route.CRITICAL_NETS), rec)
        self.assertEqual(route.CRITICAL_NETS, list(B.NETCLASSES['Motor']['nets']) + route.DRIVER_BUNDLE)

    def test_dsn_marks_in1_as_a_plane(self):
        """Hand-over 1: the board's In1.Cu is a power (plane) layer, so the Specctra export marks it (type power) and Freerouting never
        routes on it (it sets routable=false on every non-signal layer); F.Cu, In2.Cu and B.Cu stay signal layers."""
        with tempfile.TemporaryDirectory() as t:
            b = copy_board(t)
            self.assertTrue(pcbnew.ExportSpecctraDSN(b, str(Path(t) / 'x.dsn')))
            dsn = (Path(t) / 'x.dsn').read_text(encoding='utf-8')
        types = dict(re.findall(r'\(layer (\S+)\s+\(type (\w+)\)', dsn))
        self.assertEqual(types, {'F.Cu': 'signal', 'In1.Cu': 'power', 'In2.Cu': 'signal', 'B.Cu': 'signal'})

    # --- R15: no In1 slot over 10 mm on the routed board (through vias add antipads) ---
    def test_in1_has_no_slot_longer_than_10mm(self):
        """R15 on the routed, filled board: every void of the In1 GND plane - antipads of non-GND through-hole pads and of every through
        via, merged wherever a web between them is thinner than the plane's minimum width - is at most 10 mm long, measured along its
        bends (zone_check.in1_voids: the void's geodesic diameter, so an L- or U-shaped void counts its whole path)."""
        voids = zone_check.in1_voids(self.b)
        self.assertGreater(len(voids), 100)
        worst = voids[0]
        print(f'\nlargest In1 void {worst[0]:.2f} mm, bbox {tuple(round(x, 2) for x in worst[1])}; {len(voids)} voids')
        self.assertEqual([v for v in voids if v[0] > IN1_SLOT_MAX], [])

    # --- hand-over 3: B.Cu tracks keep each trunk pin's spokes live (pockets stay on their stitching vias) ---
    def test_trunk_pin_spokes_survive_routing(self):
        """PB12 on the routed, filled board: every POWER_TRUNK pin's spokes carry its pad current (board_spec.pad_current) into its
        net's trunk copper (zone_check.pin_spoke_capacity, B6: a max-flow in which a pocket or patch reached only through vias counts up to
        the via barrels and a piece cut off from the trunk counts nothing, so a B.Cu track that cuts a pocket off its stitching via is
        caught). board_spec.PAD_PENDING pins (a controller ruling pending) are the only exceptions, and must still be short."""
        short = []
        for net, pins in B.POWER_TRUNK.items():
            for ref, pin in pins:
                cap, _ = zone_check.pin_spoke_capacity(self.b, ref, pin)
                if cap < B.pad_current(ref, pin) - 1e-9: short.append((ref, pin, round(cap, 2), B.pad_current(ref, pin)))
        self.assertEqual([s for s in short if s[:2] not in B.PAD_PENDING], [])
        self.assertEqual(sorted(s[:2] for s in short if s[:2] in B.PAD_PENDING), sorted(B.PAD_PENDING))

    # --- R17a: vias keep 0.5 mm from other nets' pads ---
    def test_vias_clear_of_other_net_pads(self):
        gaps = zone_check.via_pad_gaps(self.b)
        print(f'\nmin via to other-net pad: {gaps[0]}')
        self.assertEqual([g for g in gaps if g[0] < VIA_PAD_MIN - 1e-6], [])

    # --- R17b: M3 washers (H1-H4) touch nothing but GND ---
    def test_no_copper_but_gnd_near_mounting_holes(self):
        self.assertEqual(zone_check.hole_intrusions(self.b), [])

    # --- R17d: the routed board's footprints are where board_spec puts them ---
    def test_placement_matches_board_spec(self):
        fps = {fp.GetReference(): fp for fp in self.b.GetFootprints()}
        self.assertEqual(set(fps), set(B.PLACEMENT))
        bad = []
        for ref, (u, v, rot) in B.PLACEMENT.items():
            p, r = fps[ref].GetPosition(), fps[ref].GetOrientationDegrees()
            if abs(TOMM(p.x) - u) > 1e-3 or abs(TOMM(p.y) - v) > 1e-3 or abs((r - rot + 180) % 360 - 180) > 1e-3:
                bad.append((ref, (u, v, rot), (round(TOMM(p.x), 3), round(TOMM(p.y), 3), r)))
        self.assertEqual(bad, [])

    # --- ruling (a), bay B5 per layer: no other net's copper inside a power zone polygon or a GND patch ---
    def test_no_foreign_copper_in_power_zones(self):
        """Every POWER_ZONES entry on each of its layers (trunks and helper pours) and every F.Cu GND patch (board_spec.GND_PATCHES): no
        track or via of another net has copper inside the polygon on that layer (route.py makes them router keepouts on their layers).
        Inside a helper pour (trunk=False) the scripted pre-route vias are allowed - the B4 3V3 pour under the Pico flows round the LP12
        GND vias of the pull-ups; ktest_pcb keeps every pre-route via off the trunk polygons - and nothing else."""
        zones = ([(z['net'], L, z['poly'], z.get('trunk', True)) for z in B.POWER_ZONES for L in z['layers']]
                 + [('GND', 'F.Cu', p, False) for p in B.GND_PATCHES])
        self.assertEqual({L for _, L, _, _ in zones}, {'F.Cu', 'In2.Cu', 'B.Cu'})
        scripted = {(n, (round(u, 3), round(v, 3))) for n, (u, v) in B.PREROUTE_VIAS}
        bad = []
        for net, L, pts, trunk in zones:
            lid, zone = zone_check.LAYER[L], polyset(pts)
            for item in self.tracks + self.vias:
                if item.GetNetname() == net or not item.IsOnLayer(lid): continue
                if not trunk and item.GetClass() == 'PCB_VIA' and item.IsLocked() and (item.GetNetname(), (
                        round(TOMM(item.GetPosition().x), 3), round(TOMM(item.GetPosition().y), 3))) in scripted: continue
                x = item_poly(item, lid); x.BooleanIntersection(zone)
                if x.OutlineCount(): bad.append((item.GetClass(), item.GetNetname(), net, L, round(zone_check.area_mm2(x), 3)))
        self.assertEqual(bad, [])

    # --- scripted pre-routes are on the board, locked (so Freerouting kept them) ---
    def test_preroutes_present_and_locked(self):
        have = {(t.GetNetname(), t.GetLayer(), round(TOMM(t.GetWidth()), 3),
                 frozenset({(round(TOMM(t.GetStart().x), 3), round(TOMM(t.GetStart().y), 3)),
                            (round(TOMM(t.GetEnd().x), 3), round(TOMM(t.GetEnd().y), 3))})): t.IsLocked() for t in self.tracks}
        for r in B.PREROUTES:
            layer = zone_check.LAYER[r['layer']]
            for a, b in zip(r['pts'], r['pts'][1:]):
                key = (r['net'], layer, r['width'], frozenset({tuple(map(float, a)), tuple(map(float, b))}))
                self.assertTrue(have.get(key), f'pre-route {key} missing or unlocked')
        vias = {(v.GetNetname(), round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)): v.IsLocked() for v in self.vias}
        for net, (u, v) in B.PREROUTE_VIAS:
            self.assertTrue(vias.get((net, float(u), float(v))), f'pre-route via {net} at {(u, v)} missing or unlocked')

    # --- net classes (R12) still apply to the routed board; the router saw them (R16: Motor clearance) ---
    def test_net_classes_still_apply(self):
        for net, cls in (('BAT+', 'Power'), ('F2_OUT', 'Power'), ('VMOT', 'PowerTap'), ('5V', 'PowerTap'), ('L_M1', 'Motor'),
                         ('SDA', 'Default')):
            self.assertEqual(self.b.FindNet(net).GetNetClassName(), cls, net)
        classes = {c['name']: c for c in json.loads(PRO.read_text(encoding='utf-8'))['net_settings']['classes']}
        for name, c in B.NETCLASSES.items():
            self.assertAlmostEqual(classes[name]['track_width'], c['width'])
            self.assertAlmostEqual(classes[name]['clearance'], c['clearance'])

    def test_dsn_carries_the_class_clearances(self):
        with tempfile.TemporaryDirectory() as t:
            b = copy_board(t)
            self.assertTrue(pcbnew.ExportSpecctraDSN(b, str(Path(t) / 'x.dsn')))
            dsn = (Path(t) / 'x.dsn').read_text(encoding='utf-8')
        for name, c in B.NETCLASSES.items():
            m = re.search(r'\(class ' + name + r' .*?\(clearance ([\d.]+)\)', dsn, re.S)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), c['clearance'] * 1000, msg=name)


if __name__ == '__main__':
    unittest.main()
