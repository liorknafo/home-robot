"""Bay B8: pcb/board_cad.json (pcb/export_cad.py, read from the routed board) and the robot CAD that consumes it (cad/build_robot.py).

Pure: JSON only. BoardCadTests check board_cad.json against board_spec (placement parity, frame, holes, every THT pad, the edge connectors,
the module sockets, the Pico USB receptacle) and against board_outline.json (hole count, outline area within 1 %). RobotCadTests read the
CAD outputs (cad/validation.json, cad/component-manifest.json, cad/printability.json) and re-derive the B8 facts independently: the real
board and modules sit where board_cad.json puts them, the ribs keep 1.0 mm off every THT pad, every rear-edge connector has a rib and a
fixing within 35 mm, and the B8 validation keys are true."""
import json, math, unittest
from pathlib import Path
from pcb import board_spec as B
D = Path(B.__file__).parent
BC_PATH = D / 'board_cad.json'
OUTLINE = json.loads((D / 'board_outline.json').read_text())
CAD = D.parent / 'cad'
DIRS = {'+u': (1.0, 0.0), '-u': (-1.0, 0.0), '+v': (0.0, -1.0), '-v': (0.0, 1.0)}      # board entry direction -> robot (x, y) unit vector
REAR_ROW = ('J1', 'J2', 'J6', 'J7', 'J12', 'J13')
SOCKET_H, HEADER_H = 8.5, 2.5          # female socket body (SOCKET_MPN) + the module's male header plastic: module PCB underside over the board top


def area(poly): return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]))) / 2
def robot(uv): return B.board_to_robot(*uv)
def close(a, b, tol=0.05): return all(abs(p - q) <= tol for p, q in zip(a, b))
def box_gap(a, b):
    """Plan gap between two (x0, y0, x1, y1) boxes (0 when they touch or overlap)."""
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0.0), max(a[1] - b[3], b[1] - a[3], 0.0))
def robot_box(uv_box):
    (xa, ya), (xb, yb) = robot(uv_box[:2]), robot(uv_box[2:])
    return (min(xa, xb), min(ya, yb), max(xa, xb), max(ya, yb))


class BoardCadTests(unittest.TestCase):
    """board_cad.json, as pcb/export_cad.py writes it from pcb/r1_power_io.kicad_pcb (robot frame, mm)."""
    @classmethod
    def setUpClass(cls): cls.bc = json.loads(BC_PATH.read_text())

    def test_frame_is_the_board_spec_frame_and_the_step_offset(self):
        f = self.bc['frame']
        self.assertEqual((f['board_x0'], f['board_y0']), (B.BOARD_X0, B.BOARD_Y0))
        self.assertEqual((f['z_bottom'], f['z_top'], f['thickness']), (B.Z_PCB_BOTTOM, B.Z_PCB_TOP, B.PCB_T))
        self.assertEqual(f['step_to_robot_mm'], [B.BOARD_X0, B.BOARD_Y0, B.Z_PCB_BOTTOM])      # robot = STEP + (-66.5, 50, 35.4)
        self.assertEqual(self.bc['step'], 'pcb/fab/r1_power_io.step')
        self.assertEqual(f['lead_trim_mm'], 1.4)                                                 # THT leads trimmed to <= 1.4 mm (spec)

    def test_hole_count_and_places(self):
        """Brief step 1: the hole count. Four M3 holes (Ø3.2) on board_outline holes_uv, two clamp holes Ø3.0 NPTH (ruling PB3) on
        clamp_holes_uv, in robot x, y."""
        self.assertEqual([h['ref'] for h in self.bc['holes']], ['H1', 'H2', 'H3', 'H4'])
        self.assertEqual([h['ref'] for h in self.bc['clamp_holes']], ['H5', 'H6'])
        for h, uv in zip(self.bc['holes'], OUTLINE['holes_uv']):
            self.assertTrue(close(h['xy'], robot(uv), 0.01), h); self.assertAlmostEqual(h['d'], 3.2, 3)
        for h, uv in zip(self.bc['clamp_holes'], OUTLINE['clamp_holes_uv']):
            self.assertTrue(close(h['xy'], robot(uv), 0.01), h); self.assertAlmostEqual(h['d'], 3.0, 3)

    def test_outline_area_within_one_percent(self):
        """Brief step 1: the outline area against the board (board_outline.json, the outline build_pcb drew) within +-1 %."""
        a, ref = area(self.bc['outline_xy']), area(OUTLINE['outline_uv'])
        self.assertLess(abs(a - ref) / ref, 0.01, (a, ref))
        self.assertAlmostEqual(self.bc['outline_area_mm2'], a, delta=0.01 * ref)
        xs, ys = zip(*self.bc['outline_xy'])
        self.assertTrue(-66.5 <= min(xs) <= -66.3 and 7.4 <= max(xs) <= 7.5 + 1e-6, (min(xs), max(xs)))
        self.assertAlmostEqual(min(ys), -50.0, 2); self.assertAlmostEqual(max(ys), 50.0, 2)

    def test_every_fitted_tht_pad_is_listed_at_its_place(self):
        """Every plated pad of every fitted through-hole part (and the fuse holders' NPTH pegs) is in tht_pads at board_spec's place
        (PLACEMENT parity, robot frame) with its bounding box; the mounting holes' via rings carry no lead and are not listed."""
        got = {}
        for p in self.bc['tht_pads']: got.setdefault((p['ref'], p['pin']), []).append(p)
        want = 0
        for ref, part in B.PARTS.items():
            if part['mount'] != 'tht' or part.get('dnp') or ref.startswith('H'): continue
            for pin, (x, y) in B._footprint_geometry(part['fp'])[1].items():
                if not pin: continue
                want += 1
                rows = got.get((ref, pin))
                self.assertTrue(rows, (ref, pin))
                self.assertTrue(any(close(r['xy'], robot(B.pad_uv(ref, pin)), 0.02) for r in rows), (ref, pin))
        self.assertGreaterEqual(len(self.bc['tht_pads']), want)
        self.assertFalse([p for p in self.bc['tht_pads'] if p['ref'].startswith(('H', 'TP')) or p['ref'] in ('J14', 'J15')])
        for p in self.bc['tht_pads']:
            x0, y0, x1, y1 = p['bbox_xy']
            self.assertTrue(x0 < p['xy'][0] < x1 and y0 < p['xy'][1] < y1, p)
        self.assertTrue(any(p['pin'] == '' and p['ref'] == 'F1' for p in self.bc['tht_pads']), 'the fuse holder pegs')

    def test_edge_connectors(self):
        """Every board_spec.EDGE_CONNECTORS ref: courtyard (robot box, = board_spec.courtyard_box), centre, entry direction (robot unit
        vector), family, vertical or not, mated height (MATED_H), the wire bend over a vertical plug (CABLE_BEND_MM), the lay-flat reach
        (MATED_REACH), its pins and the under-carrier flag."""
        self.assertEqual(set(self.bc['connectors']), set(B.EDGE_CONNECTORS))
        for ref, d in B.EDGE_CONNECTORS.items():
            c = self.bc['connectors'][ref]
            self.assertTrue(close(c['courtyard_xy'], robot_box(B.courtyard_box(ref)), 0.05), ref)   # KiCad's courtyard polygon keeps the 0.05 mm line: +0.045 a side
            x0, y0, x1, y1 = c['courtyard_xy']
            self.assertTrue(close(c['centre_xy'], ((x0 + x1) / 2, (y0 + y1) / 2), 1e-6), ref)
            self.assertEqual(c['entry'], d); self.assertEqual(tuple(c['entry_xy']), DIRS[d], ref)
            fam = B.connector_family(ref); vertical = B.PARTS[ref]['fp'].endswith('_Vertical')
            self.assertEqual((c['family'], c['vertical']), (fam, vertical), ref)
            self.assertEqual(c['mated_h'], B.MATED_H[ref], ref)
            self.assertEqual(c['bend'], B.CABLE_BEND_MM[fam] if vertical else 0.0, ref)
            self.assertEqual(c['mated_reach'], B.MATED_REACH.get(ref, 0.0), ref)
            self.assertEqual(c['under_carrier'], B.UNDER_CARRIER(ref), ref)
            self.assertEqual(len(c['pins_xy']), len([p for p in B._footprint_geometry(B.PARTS[ref]['fp'])[1] if p]), ref)
            bx0, by0, bx1, by1 = c['body_xy']                       # F.Fab body, inside the courtyard
            self.assertTrue(x0 - 1e-6 <= bx0 < bx1 <= x1 + 1e-6 and y0 - 1e-6 <= by0 < by1 <= y1 + 1e-6, ref)

    def test_module_sockets_and_bodies(self):
        """The four plug-in modules: every socket pin at pad_uv, the module body (board_spec.MODULES on its first socket, robot polygon)
        and the module PCB underside over the board top (socket body + header plastic)."""
        firsts = {'Pico': 'U1A', 'DRV_L': 'U3L', 'DRV_R': 'U4L', 'IMU': 'U2'}
        self.assertEqual(set(self.bc['modules']), set(firsts))
        for name, first in firsts.items():
            m, spec = self.bc['modules'][name], B.MODULES[name]
            self.assertEqual(set(m['sockets']), set(spec['rows']))
            for sock, pins in m['sockets'].items():
                self.assertEqual(len(pins), 7 if sock[:2] in ('U3', 'U4') else (20 if sock.startswith('U1') else 8), sock)
                for pin, xy in pins.items(): self.assertTrue(close(xy, robot(B.pad_uv(sock, pin)), 0.02), (sock, pin))
            body = [robot(B.local_to_board(B.PLACEMENT[first], p)) for p in B.rect_pts(spec['body'])]
            self.assertTrue(all(close(a, b, 0.02) for a, b in zip(m['body_xy'], body)), name)
            self.assertAlmostEqual(m['pcb_underside_z'], B.Z_PCB_TOP + SOCKET_H + HEADER_H, 3)
            self.assertEqual(m['h'], B.PARTS[first]['h'])

    def test_motor_cable_constants_for_the_cad(self):
        """B3 fix 1's modelled motor-cable constants, in robot terms, for the CAD to re-assert against its real route (carry item 1): the exit on
        the motor's inner end face (MOTOR_CABLE_EXIT), the band beside that face on the board side (MOTOR_CABLE_W) and the section (W x T)."""
        self.assertEqual(set(self.bc['motor_cables']), {'J4', 'J5'})
        for ref, (eu, ev, eh) in B.MOTOR_CABLE_EXIT.items():
            m = self.bc['motor_cables'][ref]
            self.assertTrue(close(m['exit_xyz'], (*robot((eu, ev)), B.Z_PCB_TOP + eh), 1e-6), ref)
            face_y = robot((eu, ev))[1]
            band = sorted((face_y, face_y - math.copysign(B.MOTOR_CABLE_W, face_y)))
            self.assertTrue(close(m['band_y'], band, 1e-6), ref)
            self.assertEqual((m['w'], m['t']), (B.MOTOR_CABLE_W, B.MOTOR_CABLE_T))

    def test_pico_usb_receptacle_at_the_left_edge(self):
        """Spec: the Pico's micro-USB plugs in out of the board's left edge (robot y +50) into the old strip-end space x -54..-42; the
        receptacle face centre sits at the Pico body's USB end, on the Pico's axis, 1.26 mm over the Pico PCB top (Pico-R3.step)."""
        u = self.bc['pico_usb']
        x, y, z = u['face_centre_xyz']
        self.assertTrue(-54.0 <= x <= -42.0, x)
        self.assertLess(abs(y - B.BOARD_Y0), 0.5, y)
        self.assertEqual(tuple(u['out_xy']), (0.0, 1.0))
        pico = self.bc['modules']['Pico']
        self.assertAlmostEqual(z, pico['pcb_underside_z'] + 1.0 + 1.26, 2)
        self.assertAlmostEqual(y, max(p[1] for p in pico['body_xy']), 2)


class RobotCadTests(unittest.TestCase):
    """The robot CAD after B8: real board and modules, ribs from the real pads, rear-edge support, plug and cable checks."""
    @classmethod
    def setUpClass(cls):
        cls.bc = json.loads(BC_PATH.read_text())
        cls.v = json.loads((CAD / 'validation.json').read_text())
        cls.man = {p['id']: p for p in json.loads((CAD / 'component-manifest.json').read_text())}

    def test_b8_validation_keys_are_true(self):
        for k in ('bay_rear_connectors_supported', 'pico_usb_insert_path_clear', 'under_carrier_cable_paths_clear', 'motor_joints_serviceable'):
            self.assertIs(self.v[k], True, k); self.assertIs(self.v['summary'][k], True, k)
        for k in ('p03_clear_of_bumper_travel', 'bay_ribs_under_rear_row', 'clash_scan_empty'): self.assertIs(self.v[k], True, k)
        self.assertEqual(self.v['jetson_carrier_bottom_z'], 57.0)
        self.assertLessEqual(self.v['bay_under_carrier_max_top_z'], B.UNDER_CARRIER_Z - B.CARRIER_CLEARANCE + 1e-6)

    def test_real_board_and_modules_in_place(self):
        """bay_board is the STEP: its plan bounds are the real outline's, its bottom is the trimmed leads (1.4 mm under the board); each
        module's plan bounds cover its sockets' pins; the IDs the pcb side lists as replaced (REMOVED_CAD_IDS) still exist."""
        for i in B.REMOVED_CAD_IDS: self.assertIn(i, self.man, i)
        b = self.man['bay_board']['bounds']
        xs, ys = zip(*self.bc['outline_xy'])
        self.assertTrue(close((b[0], b[1], b[3], b[4]), (min(xs), min(ys), max(xs), max(ys)), 0.1), b)
        self.assertAlmostEqual(b[2], B.Z_PCB_BOTTOM - 1.4, 2)
        for cid, name in (('bay_pico', 'Pico'), ('bay_drv_0', 'DRV_L'), ('bay_drv_1', 'DRV_R'), ('bay_imu', 'IMU')):
            mb = self.man[cid]['bounds']
            for pins in self.bc['modules'][name]['sockets'].values():
                for x, y in pins.values(): self.assertTrue(mb[0] < x < mb[3] and mb[1] < y < mb[4], (cid, x, y))
            self.assertGreaterEqual(mb[2], B.Z_PCB_TOP + SOCKET_H - 1e-3, cid)                        # nothing of a module below its socket top
        self.assertIn('NVIDIA', self.man['jetson']['name'])

    def test_ribs_keep_one_mm_off_every_tht_pad(self):
        """The rib-to-THT-pad rule, recomputed from board_cad.json pads and the manifest rib boxes: >= 1.0 mm in plan; each rib under the
        board (top = board bottom)."""
        ribs = [p for i, p in self.man.items() if i.startswith('bay_rib_')]
        self.assertGreaterEqual(len(ribs), 2)
        for r in ribs:
            b = r['bounds']
            self.assertAlmostEqual(b[5], B.Z_PCB_BOTTOM, 3)
            for p in self.bc['tht_pads']: self.assertGreaterEqual(box_gap((b[0], b[1], b[3], b[4]), p['bbox_xy']), 1.0 - 1e-6, (r['id'], p['ref'], p['pin']))

    def test_rear_connectors_have_a_rib_and_a_fixing_within_35mm(self):
        """Review Focus 2, recomputed: every rear-edge connector has ribs under its body (x overlap with the F.Fab body, review M3) along
        >= 50 % of the body length (y), and its centre is within 35 mm of a fixing - an M3 hole or clamp hole of board_cad.json, or a rear-edge
        hold-down finger of the CAD (bay_holddown) that really bears on the board (review M1): it reaches past the board's rear edge, its
        installed underside is on the board's top face within 0.05 mm, its contact patch over the board is >= 1 mm2 and clear of every part."""
        fix = [h['xy'] for h in self.bc['holes'] + self.bc['clamp_holes']]
        rows = self.v['bay_board']['rear_support']['holddown_fingers']
        edge = min(x for x, _ in self.bc['outline_xy'])
        self.assertGreaterEqual(len(rows), 3)
        for f in rows:
            x0, x1, z0 = f['box_xyz'][0], f['box_xyz'][3], f['box_xyz'][2]
            self.assertLess(x0, edge, f)                                                               # rooted behind the board's rear edge
            self.assertGreater(x1, edge + 1.0, f)                                                      # and reaching over it
            self.assertAlmostEqual(z0, B.Z_PCB_TOP, delta=0.06)                                        # onto the board top
            self.assertLessEqual(abs(f['finger_to_board_top_mm']), 0.05, f)
            self.assertGreaterEqual(f['contact_patch_mm2'], 1.0, f)
            self.assertEqual(f['parts_under_patch'], [], f)
            fix.append(f['contact_xy'])
        self.assertLessEqual(self.v['bay_board']['rear_support']['holddown_overlap_with_board_mm3'], 0.05)
        ribs = [p['bounds'] for i, p in self.man.items() if i.startswith('bay_rib_')]
        for ref in REAR_ROW:
            c = self.bc['connectors'][ref]
            x0, y0, x1, y1 = c['body_xy']
            iv = sorted((max(y0, b[1]), min(y1, b[4])) for b in ribs if b[0] < x1 and x0 < b[3] and max(y0, b[1]) < min(y1, b[4]))
            cov, end = 0.0, -1e9
            for a, e in iv:
                a = max(a, end)
                if e > a: cov += e - a; end = e
            self.assertGreaterEqual(cov / (y1 - y0), 0.5, ref)
            self.assertLessEqual(min(math.dist(c['centre_xy'], p) for p in fix), 35.0, ref)

    def test_holddown_springs_cover_the_stack_up(self):
        """PB24 amended: each finger is a spring preloaded against the board; the design stack covers the computed RSS stack-up, there is still
        contact at the minimum stack, the nominal force is 2-5 N and the root bending stress at the maximum stack is <= 30 % of the PETG yield."""
        sp = self.v['bay_board']['rear_support']['holddown_spring']
        self.assertGreaterEqual(sp['design_stack_mm'], sp['stack_rss_mm'])
        self.assertGreater(sp['preload_mm'], sp['design_stack_mm'])
        m = sp['E2000']
        self.assertGreater(m['min']['force_N'], 0.0)
        self.assertTrue(2.0 <= m['nominal']['force_N'] <= 5.0, m['nominal'])
        self.assertLessEqual(m['max']['root_bending_MPa'], 0.30 * 50.0 + 1e-6)

    def test_motor_joints_and_pigtails(self):
        """PB25 (amended in B9 fix 1): the J4 / J5 pigtails and their inline joints: each joint unplugs >= 8 mm along its axis, the pigtail
        clears the motor face by >= 0.3 mm, the board lifts 20 mm with the joints unplugged (the pigtails ride up with it) meeting nothing, and
        the pigtail is pre-cut to its route plus its terminations plus half the pin spread (B9 fix 2; no lift slack), then trimmed after dressing; the pigtails are in the cut list and the BOM (C18)."""
        self.assertIs(self.v['motor_joints_serviceable'], True)
        mj = self.v['bay_board']['motor_joints']
        self.assertEqual(mj['board_lift_mm'], 20.0); self.assertEqual(mj['board_lift_hits'], [])
        for side in ('left', 'right'):
            r = mj['rows'][side]
            self.assertGreaterEqual(r['unplug_mm'], 8.0); self.assertEqual(r['unplug_hits'], [])
            self.assertGreaterEqual(r['pigtail_exit_bend']['to_motor_face_mm'], 0.3)
            self.assertTrue(r['pigtail']['ok']); self.assertEqual(r['pigtail_lift_hits'], [])
            self.assertIn(f'motor_joint_{side}', self.man)
        cut = {c['id']: c for c in json.loads((D.parent / 'cable-cut-list.json').read_text())}
        bom = {p['id']: p for p in json.loads((D.parent / 'parts-and-connections.json').read_text())['parts']}
        for side in ('left', 'right'):
            c = cut[f'motor_pigtail_{side}']
            self.assertEqual(c['pigtail']['precut_mm'], c['suggested_cut_mm'])                       # B9 fix 2: the cut is the pre-cut length
            self.assertGreaterEqual(c['suggested_cut_mm'], c['pigtail']['route_mm'] + sum(c['pigtail']['terminations_mm'].values()) + c['pigtail']['pin_spread_mm'] / 2 - 1e-9)
        self.assertEqual(bom['C18']['quantity'], 2)

    def test_mating_sweeps_name_the_bench_exemption(self):
        """Review / PB25: every plug can travel its engagement + 1 mm along its pins clear; the only exemptions are J4 / J5, as plugged on the
        bench before the board is installed (PB25)."""
        sw = self.v['bay_board']['plug_mating_sweeps']
        self.assertTrue(sw['ok'])
        self.assertEqual({r for r, v in sw['rows'].items() if v['exempt']}, {'J4', 'J5'})
        for r in ('J4', 'J5'): self.assertIn('plugged on the bench before the board is installed (PB25)', sw['rows'][r]['exempt'])
        for r, v in sw['rows'].items():
            if not v['exempt']: self.assertEqual(v['hits'], [], r)
        self.assertIn('motor_joint_left', sw['rows']); self.assertIn('motor_joint_right', sw['rows'])

    def test_tray_lifts_out(self):
        """Review I3: with the battery and screws out, the tray slides 2 mm aft and lifts straight out, clear of the rear cliff sensor boards."""
        t = self.v['bay_board']['tray_removal']
        self.assertTrue(t['ok']); self.assertEqual(t['hits'], [])
        self.assertIn('tray slid 2 mm aft and lifted straight out', self.man['bay_board']['note'])

    def test_rib_print_orientation_matches_its_note(self):
        """Review M4: the rib note names the orientation cad/printability.json picks for it."""
        pr = {p['part']: p for p in json.loads((CAD / 'printability.json').read_text())['parts']}
        for i in (i for i in self.man if i.startswith('bay_rib_')):
            self.assertIn(f"'{pr[i]['best_orientation']}'", self.man[i]['note'], i)

    def test_fan_bypass_keeps_a_margin(self):
        """Review M5: the fan bypass keeps >= 1 % margin under its 25 % limit, and the no-go feeler is under the gap at that limit."""
        fs = self.v['jetson_cooling']['fan_skirt']
        self.assertGreaterEqual(fs['bypass_margin_fraction'], 0.01)
        self.assertLessEqual(fs['feeler_go_nogo_mm'][1], fs['gap_at_bypass_limit_mm'])

    def test_every_edge_connector_has_a_fixing_within_35mm(self):
        """Carry item 2: the support check covers every edge connector, not only the rear row."""
        rows = self.v['bay_board']['edge_connector_support']['rows']
        self.assertEqual(set(rows), set(B.EDGE_CONNECTORS))
        for ref, r in rows.items():
            self.assertLessEqual(r['fixing_mm'], 35.0, ref)
            if r['high_plug_force']: self.assertTrue(r['bearing_ok'], ref)
        self.assertTrue(self.v['bay_board']['edge_connector_support']['ok'])

    def test_lid_sector_stls_are_bed_ready(self):
        """Deferred L2 item: the lid sectors are written in print orientation (top face on the bed, skirt up)."""
        pr = {p['part']: p for p in json.loads((CAD / 'printability.json').read_text())['parts']}
        for i in ('P_lid_front', 'P_lid_left', 'P_lid_rear', 'P_lid_right'):
            self.assertEqual(pr[i]['best_orientation'], 'as exported', i)
            self.assertTrue(pr[i]['pass'], i)

    def test_ld14_lead_aperture_is_closed_or_counted(self):
        """Deferred L2 item: the LD14 lead aperture's warm-air path is in the jetson_cooling bypass term, which stays within its limit."""
        fs = self.v['jetson_cooling']['fan_skirt']
        self.assertIn('ld14_aperture_leak_mm2', fs)
        self.assertLessEqual(fs['bypass_fraction_of_grille_open_area'], fs['bypass_limit_fraction'])
        self.assertTrue(self.v['jetson_cooling']['ok'])


if __name__ == '__main__':
    unittest.main()
