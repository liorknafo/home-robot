"""Bay B6: power-path check on the routed, filled 4-layer board (IPC-2221 at <= 10 degC rise for the ordered copper, copper.py).
Run with KiCad's Python:
    cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_power -v
The measurements are pcb/power_check.py's, which wraps the B4 / B5 ones (zone_check.trunk_capacity / pin_spoke_capacity /
narrowest_width, copper.capacity_a). Rulings: PB11 (trunk capacity at contact level, at the true narrowest copper), PB12 (each trunk pin
carries its own load, board_spec.pad_current), the B6 carry list (via barrels in the pad metric, the < 3 % physical-margin rule, per-track
tap loads, motor tracks with no via), PB20 (J1's pinout swapped; GND judged as a plane: pad metric + the battery pin's ring cut). The
polygon floor of B4 (3.0 mm corridor, zone_check.ERODE) still holds in ktest_pcb / ktest_route; the per-net IPC capacity here supersedes it
as the sizing rule."""
import math, re, shutil, tempfile, unittest
from pathlib import Path
from unittest import mock
import pcbnew
from pcb import board_spec as B, copper as C, power_check as P, zone_check as Z
D = Path(B.__file__).parent
PCB = D / 'r1_power_io.kicad_pcb'
MOTOR_NETS = tuple(B.NETCLASSES['Motor']['nets'])
MM = pcbnew.FromMM


def scratch_copy(t):
    """The committed board (with its project and DRC rules) copied into the temp dir t and loaded: mutations never touch the repo."""
    for f in (PCB, D / 'r1_power_io.kicad_pro', D / 'r1_power_io.kicad_dru'): shutil.copy(f, Path(t) / f.name)
    return pcbnew.LoadBoard(str(Path(t) / PCB.name))


class PowerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.board = pcbnew.LoadBoard(str(PCB))          # the committed board: routed, zones filled (ktest_route checks the fill is current)

    # --- brief step 1 (PB11): every power trunk carries its net current at 10 degC ---
    def test_trunks_carry_their_current_at_10C(self):
        """Every power net of CURRENT_A but the motor pairs (tracks, test_motor_tracks_meet_ipc) and GND (a plane, not a trunk - ruling
        PB20 b: test_power_pad_spokes_carry_their_current and test_battery_gnd_ring_cut): trunk_capacity sums, over the layers where one
        zone of the net joins every POWER_TRUNK pin, the IPC-2221 current of the true narrowest copper (contact level, PB11)."""
        for net, amps in B.CURRENT_A.items():
            if net in MOTOR_NETS or net == 'GND':
                continue
            cap = P.trunk_capacity(self.board, net)['capacity_a']
            self.assertGreaterEqual(cap, amps, net)

    def test_low_margin_trunks_meet_the_physical_rule(self):
        """Carry items 2 / 3: a trunk whose conservative margin (its net current through every neck, at zero reach tolerance) is under
        LOW_MARGIN is accepted only when its neck carries a known part of that current with >= PHYSICAL_MARGIN to spare. The neck's
        current is derived, not assumed: dropping one trunk pin from the net raises the narrowest width, so the neck lies on that pin's
        own access and carries only that pin's load (board_spec.pad_current) plus every tap load of the net (TAP_LOAD_A, all of them,
        to be safe). VBAT_SW: the U1B.20 - F1.2 neck isolates U5A.1 (the P03 input, 4.62 A); VMOT: the reach into U3P.1 isolates DRV_L's
        VIN (2.1 A). Every other leg (the spanning legs without the isolated pin) still carries the full conservative current."""
        neck_pin = {'VBAT_SW': ('U5A', '1'), 'VMOT': B.drv('L', 'VIN')}       # B6's derivation (task-B6-report.md, power.txt section 2)
        for net in (n for n in B.POWER_TRUNK if n != 'GND'):
            r = P.physical_check(self.board, net)
            if r['conservative_margin'] >= P.LOW_MARGIN:
                continue
            self.assertIsNotNone(r['isolated_pin'], f'{net}: the neck carries the whole net current - widen the copper')
            self.assertEqual(r['isolated_pin'], neck_pin.get(net), (net, r['derivation']))
            self.assertAlmostEqual(r['neck_current_a'], B.pad_current(*r['isolated_pin']) + P.tap_load_total(net), 6)
            self.assertGreaterEqual(r['physical_margin'], P.PHYSICAL_MARGIN, (net, r))
            self.assertGreaterEqual(r['rest_capacity_a'], B.CURRENT_A[net], (net, r))
        self.assertAlmostEqual(B.pad_current('U5A', '1'), B.P03_IN_A, 6)

    # --- brief step 1 (Review Focus 4, PB12): THT power pads joined by spokes that carry their own load ---
    def test_power_trunk_pads_include_the_gnd_pads(self):
        got = P.power_trunk_pads(self.board)
        self.assertEqual(len(got), len(set(got)))
        want = {(r, p, n) for n, pins in B.POWER_TRUNK.items() for r, p in pins}
        self.assertLessEqual(want, set(got))
        for ref in ('J1', 'J3', 'U5A', 'U5B', 'U3P', 'U4P'):
            gnd = {(ref, p.GetNumber(), 'GND') for p in self.board.FindFootprintByReference(ref).Pads() if p.GetNetname() == 'GND'}
            self.assertTrue(gnd); self.assertLessEqual(gnd, set(got), ref)

    def test_power_pad_spokes_carry_their_current(self):      # Review Focus 4
        """Every power trunk pad: its spokes, summed over the layers it meets its zones on, carry its own load (PB12: pad_current) - and
        copper reached only through vias counts only up to the via barrels (carry item 1: zone_check.pin_spoke_capacity's max-flow);
        every joint stays a thermal relief with spokes <= 2.0 mm (iron-solderable). For GND this is the first of PB20's two plane checks
        (test_battery_gnd_ring_cut is the second)."""
        for ref, pad, net in P.power_trunk_pads(self.board):
            self.assertTrue(P.pad_uses_spokes(self.board, ref, pad), f'{ref}.{pad} must be relieved (iron-solderable)')
            if (ref, pad) in B.PAD_PENDING: continue
            self.assertGreaterEqual(P.pad_spoke_capacity(self.board, ref, pad), B.pad_current(ref, pad), f'{ref}.{pad}')

    def test_pad_metric_rates_the_pins_stacked_spokes_combined(self):
        """LIO-227: a pin's spokes on its layers sit on top of each other at its pads, so the pad metric caps the flow at their combined
        IPC capacity (copper.capacity_a), not the per-layer sum. K1.3 (VMOT, relay NO): two F.Cu and two In2.Cu spokes into the trunk."""
        spokes = P.pad_spokes(self.board, 'K1', B.RELAY_PINS['NO'])
        got = P.pad_spoke_capacity(self.board, 'K1', B.RELAY_PINS['NO'])
        self.assertEqual({L: n for L, (n, _, _) in spokes.items() if n}, {'F.Cu': 2, 'In2.Cu': 2})
        self.assertAlmostEqual(got, C.capacity_a({L: n * w for L, (n, w, _) in spokes.items() if n}), 6)
        self.assertLess(got, sum(a for _, _, a in spokes.values()))
        self.assertGreaterEqual(got, B.pad_current('K1', B.RELAY_PINS['NO']))
        # LIO-232: K1.3 was the thinnest pad metric (+2.5 %); the VMOT zones' 1.5 mm spokes lift it. (J2.1, +3.6 % then, has two F.Cu
        # spokes into F1_OUT's F.Cu helper since LIO-236.)
        self.assertEqual({L: w for L, (n, w, _) in spokes.items() if n}, {'F.Cu': 1.5, 'In2.Cu': 1.5})
        self.assertGreaterEqual(got, 1.2 * B.pad_current('K1', B.RELAY_PINS['NO']))
        # LIO-236: K1.1 (F2_OUT, relay COM) likewise - the F2_OUT zone's 1.5 mm spokes (its F.Cu strip west of the pad is 0.6 mm wide, so
        # its ring cut needed them: 3.0 mm of copper at 1.2 mm, 5.30 A by the round-3 max-flow)
        com = P.pad_spokes(self.board, 'K1', B.RELAY_PINS['COM'])
        self.assertEqual({L: w for L, (n, w, _) in com.items() if n}, {'F.Cu': 1.5})
        self.assertGreaterEqual(P.pad_spoke_capacity(self.board, 'K1', B.RELAY_PINS['COM']), 1.2 * B.pad_current('K1', B.RELAY_PINS['COM']))

    def test_trunk_pins_carry_their_current_by_the_ring_cut(self):
        """LIO-236 (round-3 review): the pad metric counts a spoke into a 0.3 mm strip or a dead-end pocket at its full width (J2.1 passed
        it at +3.6 % while a pin-to-pin max-flow through its real spokes gave 4.45 A against 7.5 A). Every POWER_TRUNK pin but GND's (a
        plane, PB20: the pad metric and test_battery_gnd_ring_cut) is rated by power_check.trunk_pin_ring - the copper leaving it through
        its real spokes, pin_ring_cut's per-layer lower bound out to the relief gap + 2.5 and + 4.5 mm (LIO-238: was + 1.0 / + 2.5; LIO-239:
        less the dead-end copper past the ring, power_check.live_copper), the smaller, on a 0.05 mm whole-cell raster, capped at the spokes, rated together; only layers whose
        copper leads on to the trunk count - and must carry its pad_current with LOW_MARGIN to spare. LIO-238: ORDERING.md's figures are
        these, the thinnest by margin first."""
        got = {}
        for net in (n for n in B.POWER_TRUNK if n != 'GND'):
            for ref, pin in B.POWER_TRUNK[net]:
                r = P.trunk_pin_ring(self.board, ref, pin)
                self.assertGreaterEqual(r['total_a'], (1 + P.LOW_MARGIN) * B.pad_current(ref, pin), (ref, pin, r['rings'], r['per']))
                got[ref, pin] = net, r['total_a']
        flat = ' '.join((D / 'fab' / 'ORDERING.md').read_text(encoding='utf-8').split())
        thinnest = min(got, key=lambda k: got[k][1] / B.pad_current(*k))
        self.assertRegex(flat, re.escape(f'the thinnest is {thinnest[0]}.{thinnest[1]} ({got[thinnest][0]}) at {got[thinnest][1]:.1f} A'))
        for k in (('J1', '2'), ('J2', '1'), ('K1', B.RELAY_PINS['COM']), ('U5B', '1')):
            self.assertRegex(flat, re.escape(f'{k[0]}.{k[1]} ({got[k][0]})') + r'(?: at)? ' + re.escape(f'{got[k][1]:.1f} A'), k)
        # cross-check: where every spoke is backed (K1.3, two F.Cu and two In2 spokes), the ring cut is the spokes, less what the whole-cell
        # raster loses on each (a half cell diagonal each side and a cell of rounding), and the pad metric's figure within that
        r = P.trunk_pin_ring(self.board, 'K1', B.RELAY_PINS['NO'])
        for L, x in r['per'].items():
            n, w = x['spokes']
            self.assertLessEqual(x['cut_mm'], n * w + 1e-9, L)
            self.assertGreaterEqual(x['cut_mm'], n * (w - 2 * P.PIN_RING_GRID / math.sqrt(2) - P.PIN_RING_GRID) - 1e-9, L)
        self.assertAlmostEqual(r['total_a'], P.pad_spoke_capacity(self.board, 'K1', B.RELAY_PINS['NO']), delta=0.05 * r['total_a'])

    def test_trunk_pin_ring_sees_a_dead_end(self):
        """LIO-236: the ring cut can fail. On a scratch copy without the F1_OUT F.Cu helper round J2.1 (its zone deleted, the board
        refilled) J2.1 has its B.Cu and In2 copper only and falls short of its 7.5 A with the margin; and the VBAT_SW In2 helper round J2.2,
        which holds no via and no other VBAT_SW pin, counts for nothing. LIO-238: the helper's strip out to its stitching vias narrowed from
        1.5 to 0.5 mm (from u 5.6, 2.3 mm from J2.1) lowers J2.1's figure by its F.Cu share.
        LIO-239: copper past the ring that leads nowhere is no sink (power_check.live_copper). J2.1's F.Cu helper necks inside + 2.5 mm, but
        its dead-end south-west corner reaches 5.85 mm out, past that ring: counted as the sink it made the + 2.5 ring read 10.3 A against
        8.4 A at + 4.5; trimmed, the two rings agree."""
        r = P.trunk_pin_ring(self.board, 'J2', '2')
        self.assertNotIn('In2.Cu', r['per'])
        self.assertIn('In2.Cu', P.pad_spokes(self.board, 'J2', '2'))
        full = P.trunk_pin_ring(self.board, 'J2', '1')
        self.assertAlmostEqual(full['rings'][2.5], full['rings'][4.5], delta=0.1, msg=full['rings'])
        with tempfile.TemporaryDirectory() as t:
            b = scratch_copy(t)
            z = next(z for z in b.Zones() if z.GetZoneName() == 'F1_OUT_POUR_F')
            o = z.Outline(); o.RemoveAllContours(); o.NewOutline()
            for u, v in ((0.6, 59.86), (5.6, 59.86), (5.6, 63.0), (11.9, 63.0), (11.9, 63.5), (5.6, 63.5), (5.6, 65.6), (0.6, 65.6)):
                o.Append(MM(u), MM(v))
            pcbnew.ZONE_FILLER(b).Fill([z])
            thin = P.trunk_pin_ring(b, 'J2', '1')
            self.assertLess(thin['per']['F.Cu']['cut_mm'], 0.6, thin)
            self.assertLess(thin['total_a'], full['total_a'] - 0.3, (thin['total_a'], full['total_a']))
            for z in list(b.Zones()):
                if z.GetZoneName() == 'F1_OUT_POUR_F': b.Delete(z)        # Delete, not Remove (route.strip: KiCad 9 SWIG)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            r = P.trunk_pin_ring(b, 'J2', '1')
            self.assertNotIn('F.Cu', r['per'])
            self.assertLess(r['total_a'], (1 + P.LOW_MARGIN) * B.pad_current('J2', '1'), r)

    def test_pad_metric_counts_via_barrels(self):
        """Carry item 1: copper that reaches the plane only through vias counts only up to the barrels. U3P.2 (ruling PB21, B6 fix 2):
        its one F.Cu spoke into its GND patch and its one B.Cu spoke into the GND island of the 5V notch carry 2 x 2.73 A, but both
        pieces reach In1 only through the patch's two stitching vias: the pin's figure is its In1 spokes plus those two barrels, not the
        raw spoke sum."""
        spokes = P.pad_spokes(self.board, 'U3P', '2')
        raw = sum(a for _, _, a in spokes.values())
        got = P.pad_spoke_capacity(self.board, 'U3P', '2')
        vias = P.pocket_vias(self.board, 'U3P', '2')
        self.assertEqual({L: n for L, (n, _, _) in spokes.items() if n}, {'F.Cu': 1, 'In1.Cu': 2, 'B.Cu': 1})
        self.assertEqual(len(vias), 2)
        self.assertLess(got, raw)
        self.assertAlmostEqual(got, spokes['In1.Cu'][2] + sum(C.via_current_a(d) for d in vias), 6)

    def test_drv_l_gnd_keeps_a_margin(self):
        """Ruling PB21: U3P.2, DRV_L's motor-current return (2.1 A), keeps >= 20 % over its current (2.52 A) by the pad metric, and still
        carries its 2.1 A with 18 um via plating (IPC-6012's class 2 minimum, against the 20 um average copper.VIA_PLATING_MM)."""
        need = B.pad_current('U3P', '2')
        self.assertGreaterEqual(P.pad_spoke_capacity(self.board, 'U3P', '2'), 1.2 * need)
        in1 = P.pad_spokes(self.board, 'U3P', '2')['In1.Cu'][2]
        thin = in1 + sum(C.via_current_a(d, plating_mm=0.018) for d in P.pocket_vias(self.board, 'U3P', '2'))
        self.assertGreaterEqual(thin, need)

    def test_mounting_holes_join_layers_through_their_barrels(self):
        """B6 fix 1 (review minor 5): the plated M3 holes H1-H4 hold no lead, so the pad metric joins their layers through their barrels
        (the M3 hole and its ring of small plated holes, one land on every layer: copper.via_current_a of each drill, summed), not as
        soldered leads with no limit between layers. J3.2's metric reaches the main B.Cu GND piece, which all four holes touch."""
        _, det = Z.pin_path_capacity(self.board, 'J3', '2')
        self.assertEqual(sorted(det['holes']), ['H1', 'H2', 'H3', 'H4'])
        for h, amps in det['holes'].items():
            pads = [p for p in self.board.FindFootprintByReference(h).Pads() if p.GetNetname() == 'GND']
            self.assertTrue(pads and all(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads), h)
            self.assertAlmostEqual(amps, sum(C.via_current_a(pcbnew.ToMM(p.GetDrillSize().x)) for p in pads), 6)
            self.assertLess(amps, math.inf)

    # --- brief step 1: motor tracks (carry item 5) ---
    def test_motor_tracks_meet_ipc(self):
        for net in MOTOR_NETS:
            self.assertGreaterEqual(P.min_track_width(self.board, net), C.ipc2221_width_mm(B.CURRENT_A[net], C.COPPER_OZ['F.Cu']), net)

    def test_motor_nets_have_no_via(self):
        """Carry item 5: the four motor pairs run pin to pin on one outer layer each (B5). A layer change would need three 0.8 / 0.4 mm
        vias (copper.via_current_a), so none may appear unnoticed."""
        self.assertEqual(P.motor_vias(self.board), [])
        for net in MOTOR_NETS:
            layers = {t.GetLayerName() for t in self.board.GetTracks() if t.GetClass() != 'PCB_VIA' and t.GetNetname() == net}
            self.assertEqual(len(layers), 1, (net, layers)); self.assertTrue(layers <= {'F.Cu', 'B.Cu'}, (net, layers))
        self.assertGreaterEqual(3 * C.via_current_a(B.CLASS_VIA['Motor'][1]), B.CURRENT_A['L_M1'])

    # --- carry item 4: every tap track and via carries the loads behind it ---
    def test_tap_tracks_carry_their_load(self):
        """For each tap net (TAP_NETS), every track segment and via carries the summed load (board_spec.TAP_LOAD_A) of the pads it
        alone feeds from the net's source (the trunk zones; 3V3: the Pico's 3V3 pin), at IPC-2221 10 degC on its layer's copper (vias:
        the barrel). The NARROW_TRACKS entries' listed current covers what they really feed."""
        short, seen = [], 0
        for net in P.TAP_NETS:
            for it in P.track_loads(self.board, net):
                seen += 1
                if it['cap_a'] < it['load_a']: short.append(it)
        self.assertGreater(seen, 50)
        self.assertEqual(short, [])
        narrow = B.narrow_track_keys()
        for it in P.track_loads(self.board, 'VBAT_SW'):
            if it['key'] in narrow:
                self.assertLessEqual(it['load_a'], B.TP1_FEED['current_a'] + 1e-9, it)

    # --- the GND plane (ruling PB20 b): a plane is not a trunk ---
    def test_battery_gnd_ring_cut(self):
        """PB20 (b) replaces the brief's single-stream GND trunk test (a plane carries current over its whole width, and only In1 alone
        joins every GND pin) with two checks: every GND trunk pin's via-aware pad metric >= its PAD_CURRENT_A
        (test_power_pad_spokes_carry_their_current), and this ring cut round the battery GND pin (J1.1 since PB20): the GND copper leaving
        the pin across every ring out to just short of the nearest GND load pin carries the battery current, 7.5 A - as a LOWER BOUND
        (power_check.pin_ring_cut). Per layer the min cut of the GND copper round a disc that holds the pad alone (its edge + 0.05 mm,
        inside the relief gap), so the cut may run through the relief spokes, confined to the ring out to the load, on a whole-cell
        raster (it never rounds a neck up), capped at the layer's own spokes; the layers' counted widths rated together on their
        combined cross-section (copper.capacity_a; LIO-231 / LIO-232 - summing each layer's own IPC current gave 8.99 A for copper that
        carried 6.71 A as a stack, so J1.1's GND patch and its F2_OUT neighbour were redrawn). Any cut of the joined layers restricted to one
        layer still cuts that layer alone, and the vias only add paths, so the stack is at most what the joined copper carries. B6 fix 1:
        B6 started the disc outside the relief (pad + gap + 0.25 mm), which took the spokes' copper as the pin's own (In1 3.46 A credited
        against 2.26 A of spokes) - not a lower bound. With the outer radius fixed, a larger disc only removes closed curves, so the
        pad-edge cut is the least over every radius (one cut, no sweep)."""
        r = P.battery_gnd_ring_cut(self.board)
        self.assertAlmostEqual(r['total_a'], C.capacity_a({L: x['w'] for L, x in r['per'].items()}), 9)
        self.assertLess(r['total_a'], sum(x['a'] for x in r['per'].values()))     # the stack is rated, not the layers' sum
        for L, x in r['per'].items(): self.assertLessEqual(x['w'], min(x['cut_mm'], x['spokes'][0] * x['spokes'][1]) + 1e-9, L)
        self.assertEqual(r['pin'], B.J1_GND_PIN)
        self.assertEqual(set(r['direct']), {'F.Cu', 'In1.Cu', 'B.Cu'})
        pad = P._pad(self.board, *B.J1_GND_PIN)
        gap = min(pcbnew.ToMM(z.GetThermalReliefGap()) for z in self.board.Zones() if z.GetNetname() == 'GND')
        self.assertLess(r['r0'], pcbnew.ToMM(min(pad.GetSize().x, pad.GetSize().y)) / 2 + gap)    # the disc holds the pad, not the spokes
        spokes = P.pad_spokes(self.board, *B.J1_GND_PIN)
        for L, x in r['per'].items():
            self.assertLessEqual(x['a'], min(x['cut_a'], spokes[L][2]) + 1e-9, L)
        self.assertGreater(r['r1'], 10.0)
        self.assertGreaterEqual(r['total_a'], B.CURRENT_A['GND'], r)
        # monotone: a disc out past the relief (B6's start) never cuts less copper on any layer than the pad-edge disc
        c, away = r['centre'], P._uv(P._pad(self.board, *r['away'].split('.')))
        for L in P.GND_CUT_LAYERS:
            ras = P.copper_raster(self.board, L, west=r['r1'] - c[0] + 3.0)
            self.assertGreaterEqual(P.ring_cut_mm(ras, c, r['r0'] + 0.7, away, r['r1']), r['per'][L]['cut_mm'], L)
        # LIO-236: ORDERING.md quotes this figure
        flat = ' '.join((D / 'fab' / 'ORDERING.md').read_text(encoding='utf-8').split())
        self.assertIn(f"{r['total_a']:.1f} A against", flat)

    def test_battery_gnd_ring_cut_fails_on_a_starved_copy(self):
        """B6 fix 1: the ring cut can fail. A scratch copy whose In1 GND plane and B.Cu GND fill meet J1.1 with no spokes at all (their pad
        connection set to none, refilled) keeps the plane round the pin but leaves it only its F.Cu spokes: the ring cut falls below
        7.5 A. B6's disc (out past the relief, cell-centre raster, the layers summed) still credits the plane's copper round the pin and
        passes the copy."""
        with tempfile.TemporaryDirectory() as t:
            b = scratch_copy(t)
            for z in b.Zones():
                if z.GetZoneName() in ('GND_PLANE', 'GND_FILL'): z.SetPadConnection(pcbnew.ZONE_CONNECTION_NONE)
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            r = P.battery_gnd_ring_cut(b)
            self.assertEqual((r['per']['In1.Cu']['a'], r['per']['B.Cu']['a']), (0.0, 0.0))
            self.assertLess(r['total_a'], B.CURRENT_A['GND'], r)
            pad, away = P._pad(b, *B.J1_GND_PIN), P._uv(P._pad(b, *r['away'].split('.')))
            old_r0 = P._half(pad) + pcbnew.ToMM(Z.zone_of(b, 'GND', C.GND_PLANE_LAYER).GetThermalReliefGap()) + 0.25
            old = 0.0
            for L in P.GND_CUT_LAYERS:
                ras = P.copper_raster(b, L, 0.2, west=r['r1'] - r['centre'][0] + 3.0, whole=False)
                mm = P.raster_cut_mm(ras, r['centre'], old_r0, away, r['r1'])
                old += C.ipc2221_current_a(mm, C.COPPER_OZ[L], internal=C.is_internal(L)) if 0 < mm < math.inf else 0.0
            self.assertGreaterEqual(old, B.CURRENT_A['GND'])            # the blind spot B6's start radius had

    def test_ring_cut_is_capped_at_the_spokes(self):
        """B12a (carry 10): pin_ring_cut credits a layer no more than its own spokes. On the real board the cut through J1.1's four In1 spokes
        (4.4 mm on the whole-cell raster) is below the spokes (4 x 1.2 mm), so the cap never binds; with zone_check.pin_spokes reporting one
        In1 spoke of the same width (patched), the same cut exceeds it and the layer counts the spoke's current."""
        near, r1 = P.nearest_gnd_load(self.board, *B.J1_GND_PIN)
        def cut(): return P.pin_ring_cut(self.board, *B.J1_GND_PIN, r1, P._uv(near), layers=('In1.Cu',))['per']['In1.Cu']
        real, spokes = cut(), Z.pin_spokes
        def one_spoke(board, ref, pin):
            out = dict(spokes(board, ref, pin))
            n, w, _ = out['In1.Cu']; out['In1.Cu'] = (1, w, C.ipc2221_current_a(w, C.COPPER_OZ['In1.Cu'], internal=True))
            return out
        with mock.patch.object(Z, 'pin_spokes', one_spoke): capped = cut()
        self.assertEqual(real['cut_mm'], capped['cut_mm'])
        self.assertGreater(real['spokes'][0], 1)
        self.assertGreater(capped['cut_a'], capped['spoke_a'])          # the cap binds ...
        self.assertEqual(capped['a'], capped['spoke_a'])                # ... and the layer counts the one spoke
        self.assertLess(capped['a'], real['a'])

    def test_no_pad_is_pending(self):
        """No trunk pin is excused: PB20 fixed the battery corner, and ruling PB21 (B6 fix 2) U3P.2, whose F.Cu patch B6 fix 1 found
        0.05 mm short of its pad (board_spec.PAD_PENDING and STITCH_PENDING are empty)."""
        self.assertEqual(B.PAD_PENDING, {})
        self.assertEqual(B.STITCH_PENDING, {})

    # --- carry item 6: C3's ripple and its GND via ---
    def test_c3_ripple_within_its_rating_and_its_via(self):
        """C3 (DRV_L's local bulk cap, PB16): its share of DRV_L's PWM input ripple (power_check.c3_ripple: I x sqrt(D (1 - D)) at the
        worst duty with slow decay, split with C1 by impedance at the PWM frequency) stays within its ripple rating (the 100 kHz figure)
        and within its single LP12 GND via's barrel. LIO-227: fast decay (the off-time coasts) puts 2 I x sqrt(D (1 - D)) on the supply,
        twice the share and over the rating, so BRINGUP-NOTES.md requires slow-decay PWM."""
        r = P.c3_ripple()
        self.assertTrue(r['slow_decay'])
        self.assertLessEqual(r['c3_a'], B.C3_RIPPLE_RATING_A)
        self.assertLessEqual(r['c3_a'], C.via_current_a(B.SMD_PAD_VIA[1]))      # LIO-226: the LP12 via drills 0.3
        fast = P.c3_ripple(slow_decay=False)
        self.assertAlmostEqual(fast['c3_a'], 2 * r['c3_a'], 9)
        self.assertGreater(fast['c3_a'], B.C3_RIPPLE_RATING_A, 'fast decay is within the rating: BRINGUP-NOTES may allow it')
        self.assertLessEqual(fast['c3_a'], C.via_current_a(B.SMD_PAD_VIA[1]))
        notes = (D / 'fab' / 'BRINGUP-NOTES.md').read_text(encoding='utf-8')
        self.assertIn('slow decay', notes); self.assertIn('IN2 = inverted PWM', notes)


if __name__ == '__main__':
    unittest.main()
