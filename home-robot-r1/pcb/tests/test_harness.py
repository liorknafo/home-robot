"""Bay B9: the harness and the BOM for the bay board (parts-and-connections.json, cable-cut-list.json, written by cad/build_robot.py).

Pure: JSON only. The checks re-derive each fact from its source - board_spec (EDGE_CONNECTORS, PARTS, NETS, PICO_GPIO, the J1 pinout),
pcb/board_cad.json (connector positions), cad/component-manifest.json (the BOM codes the CAD parts carry) and cad/validation.json (the
build's motor-side checks) - and compare it with what the harness and BOM files say:
  - every edge connector of the board is in the harness table with the board part number PARTS gives it, and with its pin -> net map;
  - the Pico pin table is PICO_GPIO; every lead that ends at a board connector ends within 10 mm of it;
  - the battery lead puts GND in cavity 1 and BAT+ in cavity 2 (ruling PB20), and says so in plain words;
  - the BOM has no D24V50F5, W_motor or MDD3A line, the drivers are the Pololu 4035 carriers under a line of their own, every BOM code a
    CAD part carries is a BOM line, the bay mounting hardware and printed parts are listed, and the base cost is the sum of its lines;
  - the J4 / J5 pigtails leave the board upward through a right-angle socket, at a bend radius the wire takes, the motor-side wires do not
    overlap, the inline joints are polarity-marked and tied down, the factory cable's slack is stored (B9 carry items 2, 3, 7-11);
  - the battery-tray hold-down's lead-in covers preload + stack (item 12) and the assembly notes carry the feeler / shim check (item 6);
  - fix round 2 / B12a (N4): the pigtails are pre-cut long by the full pin spread (the six sleeves span 12.7 mm, and so do the male
    housing's cavities, so the outer wires need +-6.35 mm at each end against the drawn Ø3 bundle) and trimmed at the cavity after
    dressing, the L / J4 and R / J5 tags and the pin-1 rule are in assembly step 0, and the factory cable's coil is the measured variable
    (fixed path + 2-3 turns takes every reading from 186 to 200 mm; up to 3.5 turns for a longer cable);
  - the cut lengths are in the BOM file, with the spike section 8 table as the rough check, and the from-memory facts sit in one verify list."""
import json, math, re, unittest
from pathlib import Path
from pcb import board_spec as B
D = Path(B.__file__).parent
ROOT = D.parent
STOCK_OR_CRIMPED = ('stock', 'crimped')


def load(p): return json.loads(p.read_text(encoding='utf-8'))


def pin_nets(ref):
    """{pin: net} of a board part, from board_spec.NETS."""
    out = {}
    for net, pins in B.NETS.items():
        for r, p in pins:
            if r == ref: out[str(p)] = net
    return out


def pin1_rule(bc, ref):
    """(which end of the lay-flat header pin 1 is at, in robot terms; True when pin 1 is on the left for someone looking into the mated socket's
    mating face, along the entry direction, with its tails up), re-derived from board_cad.json's pin positions and entry vector."""
    c = bc['connectors'][ref]
    p = c['pins_xy'];ex, ey = c['entry_xy'];lx, ly = -ey, ex   # left = z x (entry direction)
    return ('rear' if p[0][0] < p[-1][0] else 'front'), (p[0][0] - p[-1][0]) * lx + (p[0][1] - p[-1][1]) * ly > 0


CABLE_READINGS_MM = (186.0, 200.0)    # what Pololu's "200 mm" factory cable may measure to the housing's rear face (controller ruling, B9 fix round 2)
COIL_TURNS = (2.0, 3.5)      # B12a (final review N4): up to 3.5 turns, for a cable over 202 mm


class HarnessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = load(ROOT / 'parts-and-connections.json')
        cls.cut = load(ROOT / 'cable-cut-list.json')
        cls.cutd = {c['id']: c for c in cls.cut}
        cls.bc = load(D / 'board_cad.json')
        cls.man = {p['id']: p for p in load(ROOT / 'cad' / 'component-manifest.json')}
        cls.v = load(ROOT / 'cad' / 'validation.json')
        cls.parts = {p['id']: p for p in cls.d['parts']}
        cls.rows = {r['ref']: r for r in cls.d.get('bay_board_connectors', [])}

    def assy(self):
        """(step 0, step 4, service) text of the assembly order, as the bay board's manifest note carries it."""
        note = self.man['bay_board']['note']
        a = note.index('Assembly order')
        text = note[a:]
        s0 = text[text.index('0. on the bench'):text.index('1. bond')]
        s4 = text[text.index('4. plug the under-carrier cables'):text.index('5. set and verify')]
        return s0, s4, text[text.index('Service:'):]

    # ---- brief step 1 -------------------------------------------------------------------------------------------------------------
    def test_every_edge_connector_is_listed_with_its_board_part_number(self):
        """Every connector ref in EDGE_CONNECTORS appears in parts-and-connections.json with the same part number as PARTS, and with the
        pin -> net map of board_spec.NETS (the harness is wired from that table)."""
        self.assertTrue(self.rows, 'parts-and-connections.json has no bay_board_connectors table')
        for ref in B.EDGE_CONNECTORS:
            self.assertIn(ref, self.rows, ref)
            self.assertEqual(self.rows[ref]['board_part'], B.PARTS[ref]['mpn'], ref)
            self.assertEqual(self.rows[ref]['pins'], pin_nets(ref), ref)
            self.assertTrue(self.rows[ref]['mates_with'], ref)

    def test_pico_pin_table_is_pico_gpio(self):
        """The pico_pins table, read as net -> GP pin, is exactly PICO_GPIO."""
        got = {}
        for row in self.d['pico_pins']:
            gps = re.findall(r'\bGP\d+\b', row[0])
            if not gps: continue
            nets = [n.strip() for n in re.findall(r'\(([^()]*)\)', row[1])[-1].split('/')]
            self.assertEqual(len(nets), len(gps), row)
            got.update(zip(nets, gps))
        self.assertEqual(got, B.PICO_GPIO)

    def test_board_leads_end_at_their_connectors(self):
        """Every cut-list entry that ends at a board connector ends within 10 mm (plan) of that connector's position in board_cad.json
        (centre_xy) - or, for a plug that stands out past the board edge (B12a: J3's Micro-Fit receptacle), within 1 mm of its far end
        (plug_end_xy), where its wires leave it - and every edge connector has its lead in the cut list. The Pico USB cable (not an edge connector) ends where its cable
        leaves the micro-B plug: on the receptacle's axis (within 1 mm of the line from the face centre along out_xy), at most a plug length
        (20 mm) out."""
        ended = set()
        for c in self.cut:
            be = c.get('board_end')
            if not be: continue
            pt = c['points'][0 if be['end'] == 'first' else -1]
            if be['ref'] == 'PICO_USB':
                u = self.bc['pico_usb'];f, o = u['face_centre_xyz'], u['out_xy']
                along = (pt[0] - f[0]) * o[0] + (pt[1] - f[1]) * o[1];across = abs((pt[0] - f[0]) * o[1] - (pt[1] - f[1]) * o[0])
                self.assertTrue(0.0 <= along <= 20.0 and across <= 1.0 and abs(pt[2] - f[2]) <= 1.0, (c['id'], along, across))
            elif 'plug_end_xy' in self.bc['connectors'][be['ref']]:          # B12a: J3's receptacle stands out past the board edge
                self.assertLessEqual(math.dist(pt[:2], self.bc['connectors'][be['ref']]['plug_end_xy']), 1.0, (c['id'], be['ref']))
            else:
                self.assertLessEqual(math.dist(pt[:2], self.bc['connectors'][be['ref']]['centre_xy']), 10.0, (c['id'], be['ref']))
            ended.add(be['ref'])
        self.assertEqual(set(B.EDGE_CONNECTORS) - ended, set())
        self.assertIn('PICO_USB', ended)

    def test_no_bom_line_names_the_dropped_parts(self):
        """No BOM line names "D24V50F5", "W_motor" or "E06" for the drivers: the regulator is the user's P03 (no D24V50F5, no optional
        regulator total), the W_motor harness is gone, E06 was the MDD3A; the drivers are 2 x Pololu 4035 DRV8874 carriers on a line of their
        own, which the CAD's bay_drv_* parts carry. The lines the bay board replaces (perfboards, WAGO buses, inline fuse holders, the
        loose relay / capacitor / transistor / resistor sets, the strip's nylon spacers H07, the XH-4 IMU set C07) are gone."""
        for p in self.d['parts']:
            text = json.dumps(p, ensure_ascii=False)
            for word in ('D24V50F5', 'W_motor', 'MDD3A', 'perfboard'):
                self.assertNotIn(word, text, p['id'])
        self.assertNotIn('optional_regulator_usd', self.d)
        gone = {'E06', 'P04', 'E10', 'E11', 'E12', 'E13', 'E14', 'P05', 'P06', 'P09', 'C07', 'C13', 'H07'}
        self.assertEqual(gone & set(self.parts), set())
        drv = [p for p in self.d['parts'] if 'DRV8874' in p['item']]
        self.assertEqual(len(drv), 1, [p['id'] for p in drv])
        self.assertEqual(drv[0]['quantity'], 2)
        self.assertIn('4035', drv[0]['item'])
        for i in ('bay_drv_0', 'bay_drv_1'): self.assertEqual(self.man[i]['bom'], drv[0]['id'], i)

    # ---- BOM consistency ----------------------------------------------------------------------------------------------------------
    def test_every_cad_bom_code_is_a_bom_line(self):
        """Every BOM code a CAD part carries (component-manifest bom, '/'-separated) is a line of the BOM."""
        for i, p in self.man.items():
            for code in filter(None, p['bom'].split('/')): self.assertIn(code, self.parts, (i, code))

    def test_bay_mounting_hardware_and_printed_parts_are_listed(self):
        """Brief additions: the rear clamp stack (2 x M2.5 x 20 mm nylon F-F standoffs, 2 x M2.5 x 12 countersunk screws) on a line the CAD
        standoffs and screws carry; the four bay inserts and M3 x 8 screws in the insert / screw lines; the board itself and its parts (the
        fab BOM, with J4 / J5's right-angle headers); the printed spacers, clamp spacers, ribs, LD14 grommet and tray hold-down springs in the
        filament line; the rib-bonding epoxy the assembly order names."""
        code = self.man['jetson_standoff_1']['bom']
        for i in ('jetson_standoff_1', 'jetson_standoff_3', 'jetson_screw_1', 'jetson_screw_3'): self.assertEqual(self.man[i]['bom'], code, i)
        line = self.parts[code]
        self.assertEqual(line['quantity'], 2)
        for s in ('M2.5 × 20', 'F-F', 'M2.5 × 12', 'countersunk'): self.assertIn(s, line['item'], s)
        self.assertIn('four bay-board spacers', self.parts['H12']['notes'])
        self.assertIn('four M3 × 8 pan head (bay board', self.parts['H13']['notes'])
        board = [p for p in self.d['parts'] if p['source'] == 'pcb/fab/ORDERING.md']
        bparts = [p for p in self.d['parts'] if p['source'] == 'pcb/fab/r1_power_io-bom.csv']
        self.assertEqual(len(board), 1); self.assertEqual(len(bparts), 1)
        self.assertIn('4-layer', board[0]['item'])
        self.assertEqual(self.man['bay_board']['bom'], f"{board[0]['id']}/{bparts[0]['id']}")
        for s in ('Amphenol 68016-106HLF', 'right-angle'): self.assertIn(s, bparts[0]['notes'])
        f01 = self.parts['F01']['notes']
        for s in ('bay_spacer', 'bay_clamp_spacer', 'bay_rib', 'P_ld14_grommet', 'hold-down'): self.assertIn(s, f01, s)
        self.assertIn('epoxy', self.man['bay_board']['note'])
        self.assertTrue([p for p in self.d['parts'] if 'epoxy' in p['item'].lower()], 'no epoxy line')

    def test_base_cost_is_the_sum_of_the_lines(self):
        """base_new_cost_usd is the sum of every line that is not a conditional buy; no line marked Buy says it is not needed."""
        self.assertAlmostEqual(self.d['base_new_cost_usd'], round(sum(p['new_cost_usd'] for p in self.d['parts'] if p['status'] != 'Conditional buy'), 2), 2)
        for p in self.d['parts']:
            if p['status'].startswith('Buy'):
                self.assertNotRegex(p['item'] + p['notes'], r'(?i)not (needed|used)', p['id'])

    # ---- carry item 1: J1 polarity --------------------------------------------------------------------------------------------------
    def test_battery_lead_polarity_follows_j1(self):
        """PB20: J1 pin 1 = GND, pin 2 = BAT+. The harness J1 pin map equals board_spec's J1_GND_PIN / J1_BAT_PIN; the battery lead (the cut-list
        lead that ends at J1) has GND in cavity 1 and BAT+ in cavity 2 and says the polarity in plain words; the battery connection row says it
        too. Every board lead's cavity -> net list in the cut list equals its connector's NETS pin map."""
        self.assertEqual(self.rows['J1']['pins'], {B.J1_GND_PIN[1]: 'GND', B.J1_BAT_PIN[1]: 'BAT+'})
        bat = [c for c in self.cut if (c.get('board_end') or {}).get('ref') == 'J1']
        self.assertEqual(len(bat), 1)
        cav = {r['cavity']: r['net'] for r in bat[0]['pinout']}
        self.assertEqual(cav, {'1': 'GND', '2': 'BAT+'})
        for s in ('cavity 1', 'cavity 2', 'reversed'): self.assertIn(s, bat[0]['notes'], s)
        row = [r for r in self.d['connectors'] if r[0] == 'J01'][0]
        text = ' '.join(row)
        for s in ('J1', 'cavity 1 = GND', 'cavity 2 = BAT+'): self.assertIn(s, text, s)
        for c in self.cut:
            be = c.get('board_end')
            if be and be['ref'] in B.EDGE_CONNECTORS:
                self.assertEqual({r['cavity']: r['net'] for r in c['pinout']}, pin_nets(be['ref']), c['id'])

    # ---- B12a (ruling PB30, final review N1): J1 / J2 / J3 cannot be cross-plugged --------------------------------------------------
    def test_battery_rocker_and_jetson_leads_fit_only_their_own_header(self):
        """The battery (J1), rocker (J2) and Jetson 5 V (J3) headers are three different connector families (board_spec.connector_family:
        XT30, JST VH, Micro-Fit 3.0), no other edge connector shares J1's or J3's family and J2 is the only VH header, so no lead fits a
        wrong one. Each lead's board-end connector is its header's mate - in the harness table (mates_with), the cut list (its notes) and
        the BOM line it names - and each lead carries its tag ("J1 BATT", "J2 ROCKER", "J3 JETSON 5V")."""
        fam = {r: B.connector_family(r) for r in ('J1', 'J2', 'J3')}
        self.assertEqual(fam, {'J1': 'XT30', 'J2': 'VH', 'J3': 'MicroFit'})
        others = {B.connector_family(r) for r in B.EDGE_CONNECTORS if r not in fam}
        self.assertFalse(others & set(fam.values()), others)
        self.assertEqual({r: self.rows[r]['family'] for r in fam}, fam)
        mate = {'J1': ('XT30U-F', 'C19'), 'J2': ('VHR-2N', 'C05'), 'J3': ('43645-0200', 'C20')}
        tag = {'J1': '"J1 BATT"', 'J2': '"J2 ROCKER"', 'J3': '"J3 JETSON 5V"'}
        hdr = {'J1': 'XT30UPB-M', 'J2': 'B2P-VH', 'J3': '43650-0200'}
        for ref, (part, code) in mate.items():
            self.assertIn(hdr[ref], B.PARTS[ref]['mpn'], ref)
            self.assertIn(part, self.rows[ref]['mates_with'], ref); self.assertIn(code, self.rows[ref]['mates_with'], ref)
            lead = self.cutd[self.rows[ref]['lead']]
            self.assertIn(part, lead['notes'], ref); self.assertIn(tag[ref], lead['notes'], ref)
            self.assertIn(part, self.parts[code]['item'], code); self.assertEqual(self.parts[code]['status'], 'Buy', code)
            for other, (p2, _) in mate.items():
                if other != ref: self.assertNotIn(p2, lead['notes'].replace('fits neither', ''), (ref, other))
        self.assertNotIn('J1', self.parts['C05']['notes'].split('(ruling PB30')[0])     # the VH housings are J2's only
        s0, _, _ = self.assy()
        for w in ('three different connector families', '"J1 BATT"', '"J2 ROCKER"', '"J3 JETSON 5V"'): self.assertIn(w, s0, w)

    # ---- carry items 2, 7, 8, 9, 10, 11, 3: the motor pigtails, joints and factory cables ---------------------------------------------
    def test_pigtails_leave_the_board_upward(self):
        """Items 2 / 7 / 11: each J4 / J5 pigtail's board end is a right-angle 0.1 in female socket whose wires leave upward (not toward the
        motor face), with every modelled bend of its wires at >= 3 x the insulation OD; the mated socket and its wire exit stay under the
        under-carrier limit and clear of the motor face and the P03 (gated, and the P03 is never a route end of the pigtail); the C18 line
        names that part (no 0.8 mm bend left). PB25 amended (fix round 1): the pigtail is crimped to length, cut = its drawn route + its drawn
        terminations (lap on the tail, sleeve past the tip, inside the crimp housing: 3-5 mm) per wire, with no lift slack,
        because the joint is unplugged before the board lifts (fix round 2 / B12a: the pigtail is pre-cut a full pin spread longer and trimmed after
        dressing: test_pigtail_precut_covers_the_pin_spread); that lift (pigtails and male housings riding up with the board) meets nothing."""
        mj = self.v['bay_board']['motor_joints']['rows']
        for side in ('left', 'right'):
            e = mj[side]['pigtail_exit']
            self.assertIn('right-angle', e['board_end_part'])
            self.assertEqual(e['wire_exit_dir'], [0, 0, 1])
            self.assertGreaterEqual(e['min_bend_radius_mm'], 3 * e['wire_od_mm'] - 1e-9)
            self.assertLessEqual(e['top_z_mm'], B.UNDER_CARRIER_Z - B.CARRIER_CLEARANCE + 1e-6)
            self.assertGreaterEqual(e['to_motor_face_mm'], 1.0)
            self.assertTrue(e['ok'])
            self.assertGreaterEqual(e['pigtail_to_p03_mm'], e['required_gap_mm'])
            self.assertGreaterEqual(e['pigtail_to_motor_mm'], e['required_gap_mm'])
            c = self.cutd[f'motor_pigtail_{side}']
            self.assertIn('p03_regulator', c['never_end'])
            pg = c['pigtail']
            self.assertIn(pg['length_source'], STOCK_OR_CRIMPED)
            t = pg['terminations_mm']
            self.assertTrue(1.0 <= t['solder_overlap_on_tail'] <= 3.0 and t['inside_sleeve_past_tail'] > 0 and 3.0 <= t['inside_male_housing'] <= 5.0, t)
            drawn = mj[side]['pigtail']['route_mm']
            self.assertAlmostEqual(pg['route_mm'], drawn, 1)                                       # the cut list's route is the drawn one
            # fix round 2: what is cut (pre-cut) is the drawn route + terminations + half the pin spread (see the pre-cut test); the finished
            # length of a wire on the bundle axis is route + terminations, and each wire is trimmed to its own length after dressing
            self.assertAlmostEqual(pg['finished_centre_mm'], drawn + sum(t.values()), delta=0.06)
            self.assertEqual(pg['precut_mm'], c['suggested_cut_mm'])
            self.assertEqual(mj[side]['pigtail_lift_hits'], [])
        self.assertEqual(self.v['bay_board']['motor_joints']['pigtail_lift_hits'], [])
        c18 = self.parts['C18']
        self.assertEqual(c18['quantity'], 2)
        self.assertIn('right-angle', c18['item'] + c18['notes'])
        self.assertNotIn('0.8 mm', c18['notes'])
        self.assertIn('upward', c18['notes'])
        for s in ('Sullins PPPC061LGBN-RC', 'Pololu 1905'): self.assertIn(s, c18['item'] + c18['notes'], s)
        for s in ('PPPC061LJBN', 'Pololu 1904', 'USD 9.95'): self.assertNotIn(s, json.dumps(self.d, ensure_ascii=False), s)   # the review's corrected part numbers / price
        self.assertTrue(c18['source'])

    def test_cavity_colours_and_functions_follow_the_net(self):
        """Review minor 7: in every board lead's pinout a GND cavity is black (or green: the motor encoder's factory GND) and says GND or minus;
        a BAT+ / 5V / 3V3 cavity is red and says plus or volts - except the motor pigtails' encoder supply, which copies the factory cable's
        colours (red motor +, black motor -, green encoder GND, blue encoder VCC) so the colours run straight across the joint."""
        factory = {'1': 'red', '2': 'black', '3': 'green', '4': 'blue', '5': 'yellow', '6': 'white'}
        for c in self.cut:
            for r in c.get('pinout', []):
                if c['id'].startswith('motor_pigtail'):
                    self.assertEqual(r['colour'], factory[r['cavity']], (c['id'], r)); continue
                if r['net'] == 'GND':
                    self.assertIn(r['colour'], ('black', 'green'), (c['id'], r))
                    self.assertTrue('GND' in r['function'] or '−' in r['function'], (c['id'], r))
                elif r['net'] in ('BAT+', '5V', '3V3'):
                    self.assertEqual(r['colour'], 'red', (c['id'], r))
                    self.assertTrue('+' in r['function'] or ' V' in r['function'], (c['id'], r))
                else:
                    self.assertNotIn(r['colour'], ('black',), (c['id'], r))

    def test_motor_side_wires_do_not_overlap(self):
        """Item 8: a wire-to-wire overlap check covers (at least) the motor-side bundle - each pigtail with its factory cable and the leads
        that pass near them - and finds no overlap."""
        ww = self.v['wire_to_wire']
        pairs = {tuple(sorted(r['pair'])) for r in ww['rows']}
        for side in ('left', 'right'):
            self.assertIn(tuple(sorted((f'motor_pigtail_{side}', f'motor_cable_{side}'))), pairs)
        self.assertEqual(ww['overlaps'], [])
        for r in ww['rows']: self.assertGreaterEqual(r['min_gap_mm'], ww['required_mm'], r['pair'])

    def test_inline_joints_are_polarity_marked_and_tied_down(self):
        """Items 9 / 10 / 3: the unkeyed inline joint has a polarity rule written on both halves (the pin-1 mark), in the C18 line and in both
        motor connection rows; each joint is held by a modelled cable tie; the factory cable's spare length is stored in a modelled coil held
        by a tie, away from the wheels, and the route then uses the 200 mm cable (no loose loop)."""
        c18 = self.parts['C18']['notes']
        self.assertIn('pin 1', c18); self.assertIn('both halves', c18)
        for rid in ('J05', 'J06'):
            text = ' '.join([r for r in self.d['connectors'] if r[0] == rid][0])
            self.assertIn('pin 1', text, rid); self.assertIn('both halves', text, rid)
        mj = self.v['bay_board']['motor_joints']['rows']
        for side in ('left', 'right'):
            ret = mj[side]['retention']
            self.assertIn(ret['tie'], self.man); self.assertTrue(ret['ok'], side)
            fc = mj[side]['factory_cable']
            self.assertTrue(CABLE_READINGS_MM[0] <= fc['route_mm'] <= CABLE_READINGS_MM[1], fc)   # fix round 2: a lower bound too
            self.assertIn(fc['slack_tie'], self.man)
            self.assertLessEqual(fc['max_abs_y_mm'], 90.0)
            self.assertTrue(fc['ok'], side)

    # ---- fix round 2 (controller ruling): per-wire spread, L / R tags, the coil as the measured variable, the meter window ------------------
    def test_pigtail_precut_covers_the_pin_spread(self):
        """R1: the pigtail is drawn as one Ø3 bundle from the pin-row centre, but the six sleeves span 5 x 2.54 = 12.7 mm along x, so the outer
        wires need +-6.35 mm against the drawn route - and (B12a, final review N4) again at the male housing's six cavities. The wires are
        therefore pre-cut to route + terminations + the full spread (rounded up to the mm), the socket end soldered first, the bundle
        dressed along its route and each wire trimmed at its own cavity: the cut list carries pin_spread_mm (re-derived here from
        board_cad's pad positions), the pre-cut length (the cut-list cut) and the finished length (centre and range); the pre-cut is at
        least route + terminations + spread and less than a mm more; the J05 / J06 rows and the harness table say 'pre-cut' and 'finished'."""
        mj = self.v['bay_board']['motor_joints']['rows']
        for side, ref in (('left', 'J4'), ('right', 'J5')):
            xs = [p[0] for p in self.bc['connectors'][ref]['pins_xy']]
            spread = max(xs) - min(xs)
            self.assertAlmostEqual(spread, 5 * 2.54, 2)
            c = self.cutd[f'motor_pigtail_{side}'];pg = c['pigtail']
            self.assertAlmostEqual(pg['pin_spread_mm'], spread, 2)
            need = pg['route_mm'] + sum(pg['terminations_mm'].values()) + spread          # B12a (N4): the wires fan out +-spread / 2 at both ends
            self.assertGreaterEqual(pg['precut_mm'], need - 1e-9, (side, pg['precut_mm'], need))
            self.assertLess(pg['precut_mm'], need + 1.0, side)                                      # rounded up to the mm, no more
            self.assertEqual(c['suggested_cut_mm'], pg['precut_mm'])
            self.assertEqual(pg['precut_mm'], int(pg['precut_mm']))
            self.assertIn('trim', pg['length_note']);self.assertIn('pre-cut', pg['length_note']);self.assertIn('finished', pg['length_note'])
            # finished lengths (B12a, N4): each wire is trimmed at its own cavity of the male housing - route + terminations +- the spread
            # (+-spread / 2 at the socket, again at the housing) - so the cut list gives the range, not a per-pin figure from one end alone
            centre = pg['route_mm'] + sum(pg['terminations_mm'].values())
            self.assertNotIn('finished_by_pin_mm', pg)
            self.assertAlmostEqual(pg['finished_centre_mm'], centre, delta=0.06)
            lo, hi = pg['finished_range_mm']
            self.assertAlmostEqual(lo, centre - spread, delta=0.06); self.assertAlmostEqual(hi, centre + spread, delta=0.06)
            self.assertLessEqual(hi, pg['precut_mm'] + 1e-9)
            self.assertIn('cavity', pg['trim'])
            # the build's own validation row says the same, and gates it
            v = mj[side]['pigtail']
            self.assertAlmostEqual(v['pin_spread_mm'], spread, 2)
            self.assertEqual(v['precut_mm'], pg['precut_mm'])
            self.assertGreaterEqual(v['precut_mm'], v['route_mm'] + v['terminations_total_mm'] + v['pin_spread_mm'] - 1e-9)
            self.assertTrue(v['ok'], side)
            # the BOM file: the J05 / J06 row and the harness table
            text = ' '.join([r for r in self.d['connectors'] if r[0] == {'left': 'J05', 'right': 'J06'}[side]][0])
            for w in ('pre-cut', 'finished', f"{pg['precut_mm']} mm"): self.assertIn(w, text, (side, w))
            row = [r for r in self.d['harness']['cut_list'] if r['id'] == f'motor_pigtail_{side}'][0]
            self.assertEqual(row['cut_mm'], pg['precut_mm']);self.assertEqual(row['precut_mm'], pg['precut_mm']);self.assertEqual(row['finished_centre_mm'], pg['finished_centre_mm'])
            self.assertEqual(row['finished_range_mm'], pg['finished_range_mm'])
        c18 = self.parts['C18']['notes']
        for w in ('pre-cut', 'trim', 'finished', 'spread'): self.assertIn(w, c18, w)

    def test_assembly_step_0_tags_the_pigtails_and_states_the_pin_1_rule(self):
        """L/R labelling: the pigtails differ in length and in pin-1 order, so step 0 tells the builder to tag them "L / J4" and "R / J5" right
        after making them; the computed pin-1 rule stays ('J4 pin 1 = rear end, J5 pin 1 = front end; red on the left looking into the
        mating face with the tails up') and is re-derived here from board_cad.json's pin-1 positions and entry directions; step 0 also gives the
        pre-cut lengths of the cut list and the solder / dress / trim / crimp order. The same step-0 text reaches the E01, H05 and P03 notes."""
        s0, _, _ = self.assy()
        self.assertIn('"L / J4"', s0);self.assertIn('"R / J5"', s0)
        self.assertRegex(s0, r'(?i)\btag\b');self.assertIn('right after', s0)
        for ref in ('J4', 'J5'):
            end, left = pin1_rule(self.bc, ref)
            self.assertTrue(left, ref)                                                              # the same 'red on the left' rule fits both headers
            other = 'front' if end == 'rear' else 'rear'
            self.assertIn(f'{ref} pin 1 = {end} end', s0, ref)
            self.assertNotIn(f'{ref} pin 1 = {other} end', s0, ref)
        self.assertNotEqual(pin1_rule(self.bc, 'J4')[0], pin1_rule(self.bc, 'J5')[0])              # the ends differ: that is why the tags
        for w in ('red', 'on the left', 'mating face', 'tails up'): self.assertIn(w, s0, w)
        for side, ref in (('left', 'J4'), ('right', 'J5')):
            self.assertIn(f"{self.cutd[f'motor_pigtail_{side}']['pigtail']['precut_mm']} mm for {ref}", s0, ref)
        for w in ('pre-cut', 'solder', 'trim', 'crimp', 'dress'): self.assertIn(w, s0, w)
        self.assertLess(s0.index('solder'), s0.index('trim'))                                      # socket end first, trim after dressing
        for i in ('E01', 'P03', 'H05'):
            self.assertIn('"L / J4"', self.parts[i]['notes'], i)

    def test_battery_lead_meter_window_is_the_pack_voltage(self):
        """A part-discharged pack reads lower than 7 V, so the meter check no longer promises '+7...8.7 V': it must read positive, about the
        pack voltage (>= 6 V), in ASSY step 0, the W01 notes and the J01 row; a reversed lead reads negative."""
        s0, _, _ = self.assy()
        j01 = ' '.join([r for r in self.d['connectors'] if r[0] == 'J01'][0])
        for name, text in (('step 0', s0), ('W01 notes', self.cutd['W01']['notes']), ('J01', j01)):
            self.assertIn('positive, about the pack voltage (≥ 6 V', text, name)
            self.assertIn('negative', text, name)
        for doc in (self.man, self.cut, self.d):
            self.assertNotIn('7…8.7', json.dumps(doc, ensure_ascii=False))

    def test_factory_cable_coil_is_the_measured_variable(self):
        """The factory cable's length is not known to better than the vendor's '200 mm', so the coil absorbs it: step 4 says to measure each
        cable from the encoder cap to the female housing's rear face (the wire entry), route the fixed path first and wind the remainder into
        a 2-3 turn coil and tie it; the service text loosens the coil tie before the inline joint is unplugged (its 8 mm travel comes out of the
        coil). The validation carries fixed_path_mm (the route outside the coil) and the coil's turn range, and cable readings of 186-200 mm
        (rear-face reading) all fit it; the slack at the housing's rear face is named for what it is."""
        _, s4, service = self.assy()
        for w in ('measure', 'rear face', 'fixed path', 'remainder', '2–3.5 turns', 'tie it'): self.assertIn(w, s4, w)
        longest = fc_max = min(self.v['bay_board']['motor_joints']['rows'][s]['factory_cable']['fixed_path_mm'] + COIL_TURNS[1] * self.v['bay_board']['motor_joints']['rows'][s]['factory_cable']['coil']['turn_length_mm'] for s in ('left', 'right'))
        self.assertGreater(longest, 202.0)                                                          # a cable over 202 mm still fits
        self.assertIn(f'up to {math.floor(fc_max):g} mm still fits', s4)
        self.assertLess(s4.index('fixed path'), s4.index('remainder'))
        self.assertIn('loosen', service);self.assertIn('coil tie', service)
        self.assertLess(service.index('loosen'), service.index('cut the joint ties'))               # the coil tie first, then the joint ties
        self.assertIn('coil', service[service.index('loosen'):service.index('cut the joint ties')])
        unplug = [self.v['bay_board']['motor_joints']['rows'][s]['unplug_mm'] for s in ('left', 'right')]
        self.assertIn(f'{unplug[0]:g} mm', service)
        for side in ('left', 'right'):
            fc = self.v['bay_board']['motor_joints']['rows'][side]['factory_cable']
            self.assertNotIn('left_for_the_housing_mm', fc)
            self.assertAlmostEqual(fc['slack_at_housing_rear_face_mm'], fc['cable_mm'] - fc['route_mm'], 1)
            fixed, turn = fc['fixed_path_mm'], fc['coil']['turn_length_mm']
            self.assertAlmostEqual(fc['route_mm'], fixed + fc['coil']['length_mm'], delta=0.11)     # the drawn route = fixed path + the coil
            self.assertAlmostEqual(turn * fc['coil']['turns'], fc['coil']['length_mm'], 0)
            self.assertEqual(tuple(fc['cable_readings_mm']), CABLE_READINGS_MM)
            self.assertEqual(tuple(fc['coil_turns_allowed']), COIL_TURNS)
            t_lo, t_hi = [(r - fixed) / turn for r in CABLE_READINGS_MM]                           # the turns each extreme reading winds
            self.assertGreaterEqual(t_lo, COIL_TURNS[0]);self.assertLessEqual(t_hi, COIL_TURNS[1])
            self.assertAlmostEqual(fc['coil_turns_needed'][0], t_lo, delta=0.05);self.assertAlmostEqual(fc['coil_turns_needed'][1], t_hi, delta=0.05)
            self.assertGreaterEqual(fc['coil_bottom_z_at_max_turns_mm'], self.man['bay_spacer_0']['bounds'][2] - 1e-6)   # 3 turns still clear the base top
            self.assertTrue(fc['range_ok'] and fc['ok'], side)
            self.assertIn(f'{fixed:.0f} mm', s4)                                                    # the fixed path is in the text

    def test_verify_list_asks_what_the_200_mm_measures(self):
        """The coil is sized for readings of 186-200 mm, to the housing's rear face; the verify list says what Pololu's '200 mm' measures
        (the housing's rear face, the wire end or the mating face) is to be measured on the motor."""
        rows = [r for r in self.d['verify'] if '200 mm' in r['item'] + r['figure'] and 'measure' in r['item'] + r['figure']]
        self.assertTrue(rows, 'no verify row asks what the 200 mm measures')
        text = ' '.join(r['item'] + ' ' + r['figure'] for r in rows)
        for w in ('Pololu', 'rear face', 'wire end', 'mating face', '186'): self.assertIn(w, text, w)
        self.assertTrue(all(r['status'] == 'from memory: verify' for r in rows))

    # ---- carry items 12, 6: hold-down ---------------------------------------------------------------------------------------------
    def test_holddown_lead_in_covers_preload_and_stack(self):
        """Item 12: the finger lead-in is at least preload + design stack + 0.1 mm, checked by the build; the tray note gives the shim remedy
        and the linear worst case; item 6: the assembly order carries the feeler check and the shim."""
        sp = self.v['bay_board']['rear_support']['holddown_spring']
        self.assertGreaterEqual(sp['lead_in_mm'], sp['preload_mm'] + sp['design_stack_mm'] + 0.1 - 1e-9)
        self.assertTrue(sp['lead_in_ok'])
        note = self.man['P_battery_tray']['note']
        for s in ('shim', 'linear worst case'): self.assertIn(s, note, s)
        assy = self.man['bay_board']['note']
        for s in ('feeler', 'shim'): self.assertIn(s, assy, s)

    # ---- brief step 3: the cut lengths in the BOM file, the spike check ----------------------------------------------------------------
    def test_cut_lengths_are_in_the_bom_file(self):
        """parts-and-connections.json carries the cut list (every cut-list lead, its route and cut length) and the spike section 8 check:
        each spike lead's final route against the spike's 'now' and '(B)' columns, the net change near the spike's +169 mm. LIO-169: the P03
        leads are left out of that bound: the spike (and the old envelope) took ~10 mm face-to-face leads, but the real HW-316E board has its
        terminals on its short ends, so they run over its top (their routes are checked against its terminals in test_board_cad)."""
        h = self.d['harness']
        rows = {r['id']: r for r in h['cut_list']}
        self.assertEqual(set(rows), set(self.cutd))
        for i, c in self.cutd.items():
            self.assertEqual(rows[i]['route_mm'], c['route_mm'], i); self.assertEqual(rows[i]['cut_mm'], c['suggested_cut_mm'], i)
        sc = h['spike_check']
        self.assertEqual(sc['spike_net_change_mm'], 169)
        self.assertAlmostEqual(sc['net_change_mm'], sum(r['final_mm'] - r['now_mm'] for r in sc['rows']), 6)
        p03 = {'W_p03_in', 'W_p03_out'}
        self.assertEqual(p03, {i for r in sc['rows'] for i in r['ids'] if 'p03' in i})
        self.assertLessEqual(abs(sum(r['final_mm'] - r['spike_b_mm'] for r in sc['rows'] if not p03 & set(r['ids']))), 60)

    # ---- carry item 5: one verify list ----------------------------------------------------------------------------------------------
    def test_from_memory_facts_are_in_one_verify_list(self):
        """Item 5: every part number board_spec marks "verify before ordering" (B9 fix 1 retagged them from "verify at B9") and every datasheet figure quoted from memory (DRV8874, the FK and FR
        capacitors with C3's case height and tolerance, the relay, the encoders, the ToF sensors) is a row of the verify list, each with its
        source and whether the repo confirms it."""
        ver = self.d['verify']
        text = ' '.join(r['item'] for r in ver)
        mpns = [p['mpn'] for p in B.PARTS.values() if 'verify before ordering' in p.get('mpn', '')] + [x['mpn'] for x in B.BOM_EXTRAS if 'verify before ordering' in x['mpn']]
        self.assertGreaterEqual(len(mpns), 10)
        for m in mpns: self.assertIn(m.split(' (')[0], text, m)
        for s in ('DRV8874', 'EEE-FK1C101P', 'EEU-FR1E471', 'G5LE-1', '4867', 'VL53L0X'): self.assertIn(s, text, s)
        fk = [r for r in ver if 'EEE-FK1C101P' in r['item']]
        self.assertTrue(any('5.8' in r['figure'] and '0.3' in r['figure'] for r in fk), fk)
        for r in ver:
            self.assertIn(r['status'], ('in the repo', 'from memory: verify'), r)
            self.assertTrue(r['source'], r)


if __name__ == '__main__':
    unittest.main()
