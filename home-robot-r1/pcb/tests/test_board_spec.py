import json
import unittest
from functools import lru_cache
from collections import Counter
from pcb import board_spec as B, copper as C, tools
from pcb.sexpr import parse, find

@lru_cache(maxsize=None)
def lib_symbols(lib):
    return {c[1]: c for c in find(parse((tools.SYMBOL_DIR / f'{lib}.kicad_sym').read_text())[0], 'symbol')}

def symbol_pins(lib_id):
    """{pin number: pin name} of a library symbol (follows 'extends')."""
    lib, name = lib_id.split(':')
    syms = lib_symbols(lib)
    s = syms[name]
    if find(s, 'extends'): s = syms[find(s, 'extends')[0][1]]
    return {str(find(p, 'number')[0][1]): str(find(p, 'name')[0][1]) for sub in find(s, 'symbol') for p in find(sub, 'pin')}

def footprint_pads(fp_id):
    """Set of numbered pad names of a library footprint (unnumbered mounting pads are ignored)."""
    flib, fname = fp_id.split(':')
    base = B.CUSTOM_FP_DIR if flib == 'R1' else tools.FOOTPRINT_DIR / f'{flib}.pretty'
    return {str(p[1]) for p in find(parse((base / f'{fname}.kicad_mod').read_text())[0], 'pad') if str(p[1])}

def footprint_pad_types(fp_id):
    """Pad types ('smd', 'thru_hole', 'np_thru_hole', ...) of a library footprint's numbered pads."""
    flib, fname = fp_id.split(':')
    base = B.CUSTOM_FP_DIR if flib == 'R1' else tools.FOOTPRINT_DIR / f'{flib}.pretty'
    return {str(p[2]) for p in find(parse((base / f'{fname}.kicad_mod').read_text())[0], 'pad') if str(p[1])}

# Vendor ground truth, transcribed from pcb/reference/*.txt (Pololu 4035 schematic + labelled top-view photo, Pololu 2851 top view).
POLOLU_4035_ROWS = {'L': ['VM', 'GND', 'EN/IN1', 'PH/IN2', 'PMODE', 'SLEEP', 'VREF'],
                    'P': ['VIN', 'GND', 'OUT1', 'OUT2', 'IMODE', 'FAULT', 'CS']}
POLOLU_2851_PINS = ['EN', 'VIN', 'GND', 'GND', 'VOUT']

class SpecTests(unittest.TestCase):
    def test_pico_gpio_map_and_constraints(self):
        """L5b: the Pico GPIO assignment lives in one table (PICO_GPIO, net -> GP pin) and NETS follow it. Hardware constraints: SDA / SCL
        on one I2C peripheral's SDA / SCL pins, the analog nets on ADC pins GP26-28, each driver's IN1 / IN2 on the A / B outputs of one
        PWM slice (GP2n / GP2n+1), no GP pin used twice; the power pins keep their meaning (3V3_OUT feeds 3V3)."""
        g = B.PICO_GPIO
        self.assertEqual(len(set(g.values())), len(g))
        for net, gp in g.items():
            self.assertIn(B.pico(gp), B.NETS[net], f'{gp} must be on {net}')
        pico_pins = {(ref, pin) for ref in ('U1A', 'U1B') for pin in map(str, range(1, 21))}
        for net, pins in B.NETS.items():          # every Pico GP pin on a net is in the table
            for p in pins:
                if p in pico_pins and net not in ('GND', '3V3'): self.assertIn(net, g, (net, p))
        i2c = {0: ({0, 4, 8, 12, 16, 20}, {1, 5, 9, 13, 17, 21}), 1: ({2, 6, 10, 14, 18, 26}, {3, 7, 11, 15, 19, 27})}
        sda, scl = int(g['SDA'][2:]), int(g['SCL'][2:])
        self.assertTrue(any(sda in a and scl in b_ and (sda // 4 == scl // 4 or {sda, scl} == {26, 27}) for a, b_ in i2c.values()),
                        (g['SDA'], g['SCL']))
        for net in ('VSENSE', 'L_CS', 'R_CS'): self.assertIn(g[net], ('GP26', 'GP27', 'GP28'), net)
        for side in ('L', 'R'):
            a, b_ = int(g[f'{side}_IN1'][2:]), int(g[f'{side}_IN2'][2:])
            self.assertEqual((a % 2, b_, a // 2), (0, a + 1, b_ // 2), f'{side}_IN1 / {side}_IN2 not on one PWM slice A / B')
        self.assertIn(B.pico('3V3_OUT'), B.NETS['3V3'])
        for name in ('GND1', 'GND2', 'GND3', 'GND4', 'GND5', 'GND6', 'GND7', 'AGND'): self.assertIn(B.pico(name), B.NETS['GND'])
        for name in ('RUN', 'ADC_VREF', '3V3_EN', 'VSYS', 'VBUS'): self.assertIn(B.pico(name), B.NC)
    def test_every_net_has_two_pins_and_no_pin_is_shared(self):
        seen = Counter(p for pins in B.NETS.values() for p in pins)
        self.assertEqual([p for p, n in seen.items() if n > 1], [])
        self.assertEqual([n for n, pins in B.NETS.items() if len(pins) < 2], [])
        self.assertEqual(set(B.NC) & set(seen), set())
    def test_every_symbol_pin_is_netted_or_nc(self):
        for ref, part in B.PARTS.items():
            lib, name = part['sym'].split(':')
            pins = B.symbol_pin_numbers(part['sym'])
            used = {p for pins_ in B.NETS.values() for (r, p) in pins_ if r == ref} | {p for (r, p) in B.NC if r == ref}
            self.assertEqual(used, pins, f'{ref}: pins not netted {pins - used}, unknown pins {used - pins}')
    def test_symbols_and_footprints_exist(self):
        for ref, part in B.PARTS.items():
            lib, name = part['sym'].split(':')
            self.assertIn(name, lib_symbols(lib), part['sym'])
            flib, fname = part['fp'].split(':')
            base = B.CUSTOM_FP_DIR if flib == 'R1' else tools.FOOTPRINT_DIR / f'{flib}.pretty'
            self.assertTrue((base / f'{fname}.kicad_mod').is_file(), part['fp'])
    def test_vendor_pinouts_recorded(self):
        self.assertEqual(set(B.DRV_PINS), {'VIN', 'GND', 'OUT1', 'OUT2', 'IN1', 'IN2', 'PMODE', 'SLEEP', 'FAULT', 'CS'})
        self.assertEqual(set(B.REG_B_PINS) >= {'VIN', 'GND', 'VOUT'}, True)
        self.assertEqual(set(B.RELAY_PINS), {'COIL_P', 'COIL_N', 'COM', 'NO', 'NC'})
    def test_power_nets_have_currents(self):
        for n in B.NETCLASS_POWER:
            self.assertIn(n, B.NETS); self.assertGreater(B.CURRENT_A[n], 0)
    def test_placement_covers_every_part(self):
        self.assertEqual(set(B.PLACEMENT), set(B.PARTS))

    # --- additions required by the controller rulings ---
    def test_removed_cad_ids_exist(self):
        manifest = json.loads((tools.PCB_DIR.parent / 'cad' / 'component-manifest.json').read_text())
        ids = {c['id'] for c in manifest}
        self.assertEqual([i for i in B.REMOVED_CAD_IDS if i not in ids], [])
        self.assertEqual([i for i, n in Counter(B.REMOVED_CAD_IDS).items() if n > 1], [], 'duplicate ids')
    def test_power_trunk_pads_on_their_net(self):
        # bay B4: the battery-side nets are copper-zone trunks too (BAT+ J1 -> F1, F1_OUT F1 -> J2, F2_OUT F2 -> K1 COM)
        self.assertEqual(set(B.POWER_TRUNK), {'BAT+', 'F1_OUT', 'VBAT_SW', 'F2_OUT', 'VMOT', '5V', 'GND'})
        for net, pads in B.POWER_TRUNK.items():
            self.assertGreaterEqual(len(pads), 2, net)
            self.assertEqual(len(set(pads)), len(pads), f'{net}: duplicate trunk pads')
            for pad in pads:
                self.assertIn(pad, B.NETS[net], f'{pad} is not on {net}')
    def test_netclasses_partition_power_nets(self):
        classes = [set(c['nets']) for c in B.NETCLASSES.values()]
        self.assertEqual(sum(len(c) for c in classes), len(set().union(*classes)), 'a net is in two classes')
        self.assertEqual(set().union(*classes) | {'GND'}, set(B.NETCLASS_POWER))
        self.assertNotIn('GND', set().union(*classes), 'GND is a copper zone, not a track class')
        for name, c in B.NETCLASSES.items():
            self.assertGreater(c['width'], 0, name); self.assertGreater(c['clearance'], 0, name)

    # --- extra cross-checks of the transcribed vendor / library data ---
    def test_drv_header_rows_match_pololu_4035(self):
        self.assertEqual(B.DRV_ROWS, {'L': 7, 'P': 7})
        vendor = {'IN1': 'EN/IN1', 'IN2': 'PH/IN2'}
        taken = []
        for name, (row, pin) in B.DRV_PINS.items():
            self.assertEqual(POLOLU_4035_ROWS[row][pin - 1], vendor.get(name, name), name); taken.append((row, pin))
        for row, pin in B.DRV_EXTRA_GND:
            self.assertEqual(POLOLU_4035_ROWS[row][pin - 1], 'GND'); taken.append((row, pin))
        for row, pin in B.DRV_UNUSED:
            self.assertIn(POLOLU_4035_ROWS[row][pin - 1], {'VM', 'IMODE', 'VREF'}); taken.append((row, pin))
        every = [(row, i + 1) for row, names in POLOLU_4035_ROWS.items() for i in range(len(names))]
        self.assertEqual(sorted(taken), sorted(every), 'every carrier pin must be in exactly one of PINS / EXTRA_GND / UNUSED')
        self.assertEqual(B.DRV_PMODE_NET, '3V3', 'PMODE high selects PWM (IN/IN) mode')
    def test_regulator_b_pins_match_pololu_2851(self):
        got = [name for name, _ in sorted(B.REG_B_PINS.items(), key=lambda kv: kv[1])]
        self.assertEqual([n[:3] if n.startswith('GND') else n for n in got], POLOLU_2851_PINS)
        self.assertEqual(sorted(B.REG_B_PINS.values()), [1, 2, 3, 4, 5])
    def test_relay_pins_match_omron_g5le1(self):
        # Omron G5LE datasheet p.3, SPDT bottom view: 1 COM, 2 / 5 coil, 3 NO, 4 NC (contact rests on 4).
        self.assertEqual({k: v for k, v in B.RELAY_PINS.items() if k in ('COM', 'NO', 'NC')}, {'COM': '1', 'NO': '3', 'NC': '4'})
        self.assertEqual({B.RELAY_PINS['COIL_P'], B.RELAY_PINS['COIL_N']}, {'2', '5'})
    def test_footprint_pads_cover_symbol_pins(self):
        for ref, part in B.PARTS.items():
            self.assertEqual(B.symbol_pin_numbers(part['sym']) - footprint_pads(part['fp']), set(), f'{ref}: symbol pins without a pad on {part["fp"]}')
    def test_polarity_and_pin_roles(self):
        self.assertEqual(symbol_pins(B.PARTS['D1']['sym'])['1'], 'K'); self.assertIn(('D1', '1'), B.NETS['COIL_P'])
        self.assertIn(('D1', '2'), B.NETS['GND'])
        for i in range(1, 4):
            self.assertEqual(symbol_pins(B.PARTS[f'LED{i}']['sym'])['1'], 'K'); self.assertIn((f'LED{i}', '1'), B.NETS['GND'])
        for i in range(1, 6):   # 2N7002 SOT-23 (KiCad Transistor_FET:2N7002): 1 = G, 2 = S, 3 = D
            self.assertEqual(symbol_pins(B.PARTS[f'Q{i}']['sym']), {'1': 'G', '2': 'S', '3': 'D'}); self.assertIn((f'Q{i}', '2'), B.NETS['GND'])
        self.assertIn(('C1', '1'), B.NETS['VMOT']); self.assertIn(('C1', '2'), B.NETS['GND'])   # Device:C_Polarized pin 1 is '+'
    def test_fuse_holder_is_a_standard_library_part(self):
        self.assertEqual(B.FUSE_FP, 'Fuse:FuseHolder_Blade_ATO_Littelfuse_FLR_178.6165')
        self.assertEqual(footprint_pads(B.FUSE_FP), {'1', '2'})
        self.assertFalse((B.CUSTOM_FP_DIR).exists(), 'no custom footprint library needed')
        for ref in ('F1', 'F2'): self.assertEqual(B.PARTS[ref]['mpn'], 'Littelfuse 178.6165.0001')
    def test_part_heights_fit_under_the_lid(self):
        for ref, part in B.PARTS.items():
            self.assertLessEqual(part['h'], B.MAX_PART_H, ref)

    # --- Task B2: the bay board (spec 2026-10-08-r1-bay-board), 74 x 100 mm under the Jetson ---
    def test_bay_frame(self):
        """Robot x -66.5..7.5, y -50..50 (u = x + 66.5 toward the front, v = 50 - y toward the right), board bottom z 35.4 on 2.4 mm spacers."""
        self.assertEqual((B.BOARD_X0, B.BOARD_Y0, B.BOARD_W, B.BOARD_H), (-66.5, 50.0, 74.0, 100.0))
        self.assertEqual((B.Z_PCB_BOTTOM, B.Z_PCB_TOP), (35.4, 37.0))
        self.assertEqual(B.PCB_T, 1.6)
        self.assertLessEqual(max(B.BOARD_W, B.BOARD_H), 100.0)   # JLCPCB <= 100 x 100 price
        self.assertAlmostEqual(B.UNDER_CARRIER_MAX_H, 16.27, places=2)
        self.assertAlmostEqual(B.MAX_PART_H, 60.5, places=2)
        self.assertEqual(B.robot_to_board(-66.5, 50.0), (0.0, 0.0)); self.assertEqual(B.robot_to_board(7.5, -50.0), (74.0, 100.0))
        self.assertEqual(B.board_to_robot(*B.robot_to_board(-33.0, 40.0)), (-33.0, 40.0))
        self.assertEqual(B.CARRIER_U, 28.0); self.assertEqual(B.robot_to_board(-38.5, 0.0)[0], B.CARRIER_U)    # the Jetson carrier's rear edge, x -38.5
    def test_max_part_h_formula(self):
        """Behind the carrier a part may rise to the lid underside less 3 mm (z 97.5); under it to the carrier's lowest lead (z 54.77) less 1.5 mm."""
        self.assertEqual((B.LID_UNDERSIDE_Z, B.CEILING_Z_LID), (100.5, 97.5))
        self.assertAlmostEqual(B.CEILING_Z_LID, B.LID_UNDERSIDE_Z - 3.0, 6)
        self.assertAlmostEqual(B.MAX_PART_H, B.CEILING_Z_LID - B.Z_PCB_TOP, 6)
        self.assertEqual((B.UNDER_CARRIER_Z, B.CARRIER_CLEARANCE), (54.77, 1.5))
        self.assertAlmostEqual(B.UNDER_CARRIER_MAX_H, B.UNDER_CARRIER_Z - B.CARRIER_CLEARANCE - B.Z_PCB_TOP, 6)
        self.assertLess(B.UNDER_CARRIER_MAX_H, B.MAX_PART_H)
    def test_height_zones_are_well_formed(self):
        self.assertEqual(B.HEIGHT_ZONES, [(28.0, 0.0, 74.0, 100.0, B.UNDER_CARRIER_MAX_H)])      # x >= -38.5: under the Jetson carrier, to both side edges and the front edge
        for u0, v0, u1, v1, h in B.HEIGHT_ZONES:
            self.assertTrue(0 <= u0 < u1 <= B.BOARD_W and 0 <= v0 < v1 <= B.BOARD_H, (u0, v0, u1, v1))
            self.assertTrue(0 < h < B.MAX_PART_H, h)
        self.assertEqual(B.max_part_h_at(10.0, 50.0), B.MAX_PART_H)                # behind the carrier
        self.assertEqual(B.max_part_h_at(27.9, 0.0), B.MAX_PART_H); self.assertEqual(B.max_part_h_at(27.9, 100.0), B.MAX_PART_H)
        self.assertEqual(B.max_part_h_at(40.0, 50.0), B.UNDER_CARRIER_MAX_H)       # under the carrier
        self.assertEqual(B.max_part_h_at(28.0, 0.0), B.UNDER_CARRIER_MAX_H)        # zone edges belong to the zone
        self.assertEqual(B.max_part_h_at(74.0, 100.0), B.UNDER_CARRIER_MAX_H)
    def test_the_old_carrier_band_rule_is_retired(self):
        """Spec: BAND_PLAN_MARGIN / BAND_PLAN_EXEMPT are gone; the Pico is just a part behind the carrier."""
        for name in ('BAND_PLAN_MARGIN', 'BAND_PLAN_EXEMPT'): self.assertFalse(hasattr(B, name), name)
    def test_removed_cad_ids_are_the_bay_placeholders(self):
        want = {'bay_board', 'bay_pico', 'bay_drv_0', 'bay_drv_1', 'bay_imu', 'pico_usb_plug',
                *[f'bay_spacer_{i}' for i in range(4)], *[f'bay_screw_{i}' for i in range(4)], 'bay_clamp_spacer_1', 'bay_clamp_spacer_3'}
        self.assertEqual(set(B.REMOVED_CAD_IDS), want)
        self.assertEqual(len(B.REMOVED_CAD_IDS), len(want), 'duplicate ids')
        for kept in ('jetson_standoff_1', 'jetson_standoff_3', 'jetson_screw_1', 'jetson_screw_3', 'p03_regulator', 'insert_bay_0', 'bay_rib_0', 'bay_rib_1'):
            self.assertNotIn(kept, B.REMOVED_CAD_IDS)

    # --- Task L3b: small parts are SMD, reflowed (spec 2026-10-06-r1-low-profile section 3) ---
    SMD_REFS = ([f'R{i}' for i in range(1, 24)] + ['C2', 'C3', 'D1'] + [f'LED{i}' for i in range(1, 4)] + [f'Q{i}' for i in range(1, 6)]
                + [f'TP{i}' for i in range(1, 6)])
    def test_small_parts_use_standard_smd_footprints(self):
        want = {**{f'R{i}': 'Resistor_SMD:R_0805_2012Metric' for i in range(1, 24)}, 'C2': 'Capacitor_SMD:C_0805_2012Metric',
                'C3': 'Capacitor_SMD:CP_Elec_6.3x5.8',     # PB16: DRV_L's bulk capacitor, a V-chip (PB20: the 5.8 mm can)
                **{f'LED{i}': 'LED_SMD:LED_0805_2012Metric' for i in range(1, 4)}, 'D1': 'Diode_SMD:D_SMA',
                **{f'Q{i}': 'Package_TO_SOT_SMD:SOT-23' for i in range(1, 6)}, **{f'TP{i}': 'TestPoint:TestPoint_Pad_D2.0mm' for i in range(1, 6)}}
        self.assertEqual({r: B.PARTS[r]['fp'] for r in self.SMD_REFS}, want)
        for r in self.SMD_REFS:
            self.assertEqual(footprint_pad_types(B.PARTS[r]['fp']), {'smd'}, r)
    def test_mount_field_is_smd_top_for_small_parts_and_tht_for_the_rest(self):
        for ref, part in B.PARTS.items():
            self.assertIn(part['mount'], ('smd_top', 'tht'), ref)
            self.assertEqual(part['mount'] == 'smd_top', ref in self.SMD_REFS, ref)
            types = footprint_pad_types(part['fp'])
            self.assertEqual(types == {'smd'}, part['mount'] == 'smd_top', f'{ref}: mount {part["mount"]} vs pad types {types}')
        tht = {r for r, p in B.PARTS.items() if p['mount'] == 'tht'}
        self.assertEqual({r for r in tht if r[0] in 'JUFH'} | {'K1', 'C1'}, tht)    # connectors, sockets, regulator terminals, fuse holders, holes, relay, C1
        for r in ('K1', 'C1', 'F1', 'F2', 'U5A', 'U5B'): self.assertEqual(B.PARTS[r]['mount'], 'tht', r)
    def test_smd_parts_values_and_heights(self):
        for i in range(1, 24):
            self.assertIn('0805', B.PARTS[f'R{i}']['mpn']); self.assertIn('1 %', B.PARTS[f'R{i}']['mpn'])
        for i in range(1, 6): self.assertEqual((B.PARTS[f'Q{i}']['value'], B.PARTS[f'Q{i}']['sym']), ('2N7002', 'Transistor_FET:2N7002'))
        self.assertEqual((B.PARTS['D1']['value'], B.PARTS['D1']['sym']), ('SS14', 'Diode:SS14'))
        for r in self.SMD_REFS:      # PB16 / PB20: C3 (DRV_L's 100 uF V-chip, 5.8 mm) is the one tall SMD part, below MODULE_CLEARANCE
            self.assertLessEqual(B.PARTS[r]['h'], 5.8 if r == 'C3' else 3.0, f'{r}: tallest SMD bodies - C3 5.8 mm (PB20), else the SMA diode 2.7 mm')
    def test_transistor_and_diode_remap_by_symbol_pin_name(self):
        """THT 2N7000 (1 = S, 2 = G, 3 = D) -> SOT-23 2N7002 (1 = G, 2 = S, 3 = D) and 1N5819 -> SS14: the same signals sit on the same
        functional pins, checked through the pin NAMES of the library symbols rather than the numbers."""
        by_pin = {pin: net for net, pins in B.NETS.items() for pin in pins}
        def net_of(ref, name):
            names = symbol_pins(B.PARTS[ref]['sym'])
            (num,) = [n for n, nm in names.items() if nm == name]
            return by_pin[(ref, num)]
        self.assertEqual({B.PARTS[f'Q{i}']['sym'] for i in range(1, 6)}, {'Transistor_FET:2N7002'})
        self.assertEqual(symbol_pins('Transistor_FET:2N7002'), {'1': 'G', '2': 'S', '3': 'D'})
        self.assertEqual(footprint_pads('Package_TO_SOT_SMD:SOT-23'), {'1', '2', '3'})
        for i, sig in enumerate(('LA', 'LB', 'RA', 'RB'), 1):
            self.assertEqual((net_of(f'Q{i}', 'G'), net_of(f'Q{i}', 'D'), net_of(f'Q{i}', 'S')), (f'ENC_{sig}_5V', f'ENC_{sig}_3V3', 'GND'), f'Q{i}')
        self.assertEqual((net_of('Q5', 'G'), net_of('Q5', 'D'), net_of('Q5', 'S')), ('COIL_P', 'MOTOR_ON_N', 'GND'))
        self.assertEqual(B.Q_PINS, {'G': '1', 'S': '2', 'D': '3'})
        self.assertEqual(symbol_pins(B.PARTS['D1']['sym']), {'1': 'K', '2': 'A'})
        self.assertEqual((net_of('D1', 'K'), net_of('D1', 'A')), ('COIL_P', 'GND'))      # flyback: cathode on the coil +, anode on GND
        self.assertEqual(footprint_pads('Diode_SMD:D_SMA'), {'1', '2'})                  # D_SMA pad 1 = cathode (the band side)
        for i in range(1, 4): self.assertEqual((net_of(f'LED{i}', 'K'), net_of(f'LED{i}', 'A')), ('GND', f'LED{i}_A'))

    # --- Task 5: placement data (ruling R8 and the placement intent) ---
    def test_module_sockets_line_up(self):
        """R8: every socket of a plug-in module has the same rotation and sits where the module's real pin rows are."""
        self.assertEqual(module_misalignments(B.PLACEMENT), [])
    def test_real_row_spacings(self):
        self.assertEqual(B.DRV_ROW_SPACING, 12.70); self.assertEqual(B.PICO_ROW_SPACING, 17.78)
        self.assertEqual(B.MODULES['Pico']['rows'], {'U1A': (0.0, 0.0), 'U1B': (17.78, 0.0)})          # GP0 row left, VBUS row right
        for side, n in (('L', 3), ('R', 4)):
            self.assertEqual(B.MODULES[f'DRV_{side}']['rows'], {f'U{n}L': (0.0, 0.0), f'U{n}P': (12.70, 0.0)})   # VM row left, VIN row right
        self.assertEqual(set(B.MODULES), {'Pico', 'DRV_L', 'DRV_R', 'IMU'})
        self.assertEqual({r for m in B.MODULES.values() for r in m['rows']},
                         {'U1A', 'U1B', 'U2', 'U3L', 'U3P', 'U4L', 'U4P'})

    # --- Task L5: the strip placement ---
    def test_u5c_deleted(self):
        """L5 / fix 1 (ruling LP9): the D24V50F5 socket U5C is not on the strip (no clean place; the user's P03 on U5A / U5B is the
        regulator). It is gone from every circuit table; REG_B_PINS stays as the vendor record."""
        self.assertFalse(hasattr(B, 'U5C_FITTED'))
        self.assertNotIn('U5C', B.PARTS); self.assertNotIn('U5C', B.PLACEMENT); self.assertNotIn('D24V50F5', B.MODULES)
        self.assertEqual([p for pins in B.NETS.values() for p in pins if p[0] == 'U5C'], [])
        self.assertEqual([p for p in B.NC if p[0] == 'U5C'], [])
        self.assertEqual([p for pads in B.POWER_TRUNK.values() for p in pads if p[0] == 'U5C'], [])
        self.assertEqual(B.POWER_TRUNK['VBAT_SW'], [('J2', '2'), ('F2', '1'), ('U5A', '1')])
        self.assertEqual(B.POWER_TRUNK['5V'], [('U5B', '1'), ('J3', '1')])
        self.assertEqual(sorted(B.REG_B_PINS.values()), [1, 2, 3, 4, 5])
    def test_no_regulator_bay(self):
        """The P03 is off the board (spec 2026-10-06 section 2): no regulator bay."""
        self.assertFalse(hasattr(B, 'REG_BAY'))
    def test_preroutes_are_well_formed(self):
        """L6 / L5b pre-routes: each on a real net, on a copper layer of the 4-layer stack-up (bay B4), at least its net class's width, inside the board box, never GND; the
        pre-route vias are the SMD GND pads' plane vias, one per top-side part with a GND pad (ruling (c): no GND track), plus the hops'
        vias (HOP_VIAS, L5b), each a layer change: pre-routes of its own net on two layers end at it."""
        width = {n: c['width'] for c in B.NETCLASSES.values() for n in c['nets']}
        for r in B.PREROUTES:
            self.assertIn(r['net'], B.NETS); self.assertIn(r['layer'], C.STACKUP)
            if 'narrow' not in r: self.assertGreaterEqual(r['width'], width.get(r['net'], B.DEFAULT_CLASS['width']) - 1e-9, r)
            self.assertNotEqual(r['net'], 'GND')
            for u, v in r['pts']: self.assertTrue(0 < u < B.BOARD_W and 0 < v < B.BOARD_H, r)
        smd_gnd = {ref for ref, pin in B.NETS['GND'] if B.PARTS[ref]['mount'] == 'smd_top'}
        self.assertEqual(set(B.SMD_GND_VIAS), smd_gnd)
        self.assertEqual(sorted(B.PREROUTE_VIAS), sorted([('GND', uv) for uv in B.SMD_GND_VIAS.values()] + B.STITCH_VIAS + B.HOP_VIAS))
        ends = {}                                   # (net, point) -> layers of the pre-routes of the net that end there
        for r in B.PREROUTES:
            for p in (r['pts'][0], r['pts'][-1]): ends.setdefault((r['net'], tuple(p)), set()).add(r['layer'])
        for net, uv in B.HOP_VIAS:                  # a layer change: pre-routes of its net on two layers end at the via
            self.assertNotEqual(net, 'GND'); self.assertGreaterEqual(len(ends.get((net, tuple(uv)), ())), 2, (net, uv))
    def test_narrow_tracks_are_listed_probe_feeds(self):
        """PB18: a track under its class width only as a PREROUTES entry flagged `narrow` - a scripted feed of a probe pad or lamp,
        >= 0.3 mm, with its reason and current (for B6); NARROW_TRACKS is exactly those entries (one source)."""
        width = {n: c['width'] for c in B.NETCLASSES.values() for n in c['nets']}
        self.assertEqual(B.NARROW_TRACKS, [r for r in B.PREROUTES if 'narrow' in r])
        self.assertGreater(len(B.NARROW_TRACKS), 0)
        for e in B.NARROW_TRACKS:
            self.assertLess(e['width'], width.get(e['net'], B.DEFAULT_CLASS['width']), e)
            self.assertGreaterEqual(e['width'], 0.3, e); self.assertTrue(e['narrow']['reason'], e)
            self.assertGreaterEqual(e['narrow']['current_a'], 0.0, e)
        keys = B.narrow_track_keys()
        self.assertEqual(len(keys), sum(len(e['pts']) - 1 for e in B.NARROW_TRACKS))
    def test_pico_usb_end_points_at_the_left_edge(self):
        """Bay B3: the Pico's micro-USB end points -v, out of the board's left edge (v 0, robot +y: the old strip-end space, pluggable from
        above with the lid off), and the module runs along +v from there."""
        x0, y0, x1, y1 = B.MODULES['Pico']['body']
        u, v = B.local_to_board(B.PLACEMENT['U1A'], ((x0 + x1) / 2, y0))     # middle of the USB end
        far = B.local_to_board(B.PLACEMENT['U1A'], ((x0 + x1) / 2, y1))
        self.assertAlmostEqual(far[0], u, 6)
        self.assertGreater(far[1], v + 45)                                     # pointing -v: the far end lies at +v
        self.assertTrue(0.0 <= v <= 3.0, v)
    def test_every_off_board_connector_has_an_edge(self):
        """Bay B3: every connector and P03 terminal has an entry direction (EDGE_CONNECTORS: '+u' front, '-u' rear, '-v' left, '+v' right;
        the cable leaves the board that way). Rear edge: battery, rocker, toggle loop, arm button, rear cliffs (behind the carrier); tongue
        front edge (facing the P03): the front cliffs and both P03 terminals; side edges: the bumpers; the motor headers toward their
        motor notches; J3 (Jetson 5 V, ruling PB5) out of the right edge toward the W04 corridor. MATED_H / CABLE_BEND_MM / MATED_REACH
        and connector_family describe every one of them."""
        dnp = {r for r, p in B.PARTS.items() if p.get('dnp')}
        self.assertEqual(dnp, {'J14', 'J15'})                                 # PB8: the only DNP parts are the two spare rows
        conns = {r for r in {f'J{i}' for i in range(1, 16)} | {'U5A', 'U5B'} if r not in dnp}   # DNP spare rows: no edge, no cable
        self.assertEqual(set(B.EDGE_CONNECTORS), conns)
        for ref, d in B.EDGE_CONNECTORS.items(): self.assertIn(d, ('+u', '-u', '+v', '-v'), ref)
        want = {'J1': '-u', 'J2': '-u', 'J6': '-u', 'J7': '-u', 'J12': '-u', 'J13': '-u', 'J8': '+u', 'J9': '+u', 'U5A': '+u', 'U5B': '+u',
                'J10': '-v', 'J11': '+v', 'J4': '-v', 'J5': '+v', 'J3': '+v'}
        for ref, d in want.items(): self.assertEqual(B.EDGE_CONNECTORS[ref], d, ref)
        self.assertEqual(B.TERMINALS, {'U5A': '+u', 'U5B': '+u'})
        for ref in conns: self.assertEqual(B.PARTS[ref]['mount'], 'tht', ref)
        self.assertEqual(B.CABLE_BEND_MM, {'XH': 5.0, 'PinHeader': 5.0, 'VH': 7.0})
        self.assertEqual(set(B.MATED_H), conns)
        fam = {r: B.connector_family(r) for r in conns}
        self.assertEqual({r for r, f in fam.items() if f == 'Terminal'}, {'U5A', 'U5B'})
        self.assertEqual({r for r, f in fam.items() if f == 'VH'}, {'J1', 'J2', 'J3'})
        self.assertEqual({r for r, f in fam.items() if f == 'PinHeader'}, {'J4', 'J5'})
        for ref, h in B.MATED_H.items(): self.assertGreater(h, 0.0, ref)
        for ref, r in B.MATED_REACH.items():                                  # PB6: only right-angle parts whose plug lies on the board
            self.assertTrue(B.PARTS[ref]['fp'].endswith('_Horizontal'), ref); self.assertGreater(r, 0.0, ref)
        for ref in conns: self.assertIn(B.PARTS[ref]['fp'].split(':')[1], B.ENTRY_LOCAL, ref)
    def test_under_carrier_connectors_are_right_angle_or_low_enough(self):
        """Step 3: a connector whose courtyard reaches under the carrier (UNDER_CARRIER) keeps MATED_H (+ the wire bend over a vertical
        part) within UNDER_CARRIER_MAX_H; the 0.1 in motor / spare headers (14 mm housing: 16.54 mm mated) and the VH Jetson lead (16.5 mm)
        therefore use side-entry parts there."""
        for ref in B.EDGE_CONNECTORS:
            vertical = B.PARTS[ref]['fp'].endswith('_Vertical')
            need = B.MATED_H[ref] + (B.CABLE_BEND_MM[B.connector_family(ref)] if vertical else 0.0)
            if B.UNDER_CARRIER(ref): self.assertLessEqual(need, B.UNDER_CARRIER_MAX_H, ref)
        for ref in ('J4', 'J5'): self.assertEqual(B.PARTS[ref]['fp'], 'Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Horizontal', ref)
    def test_under_carrier_flag_follows_the_courtyard(self):
        """UNDER_CARRIER(ref) is True exactly when the part's courtyard (from its footprint file, placed by PLACEMENT) reaches u >= CARRIER_U."""
        for ref in B.PARTS:
            u0, v0, u1, v1 = B.courtyard_box(ref)
            self.assertLess(u0, u1, ref); self.assertLess(v0, v1, ref)
            self.assertEqual(B.UNDER_CARRIER(ref), u1 >= B.CARRIER_U, ref)
        self.assertTrue(B.UNDER_CARRIER('U3P')); self.assertFalse(B.UNDER_CARRIER('J1'))
        x0, y0, x1, y1 = B.courtyard_box('J1')
        self.assertTrue(x1 < B.CARRIER_U)
    def test_motor_cable_exit_matches_the_motor_geometry(self):
        """B3 fix 1: MOTOR_CABLE_EXIT (board u, v and height over the board top) is where each factory cable leaves its motor: on the
        motor's axis (u of the motor_left / right box centre in cad/component-manifest.json), on its inner end face (the box face nearest the
        board), at a height inside the motor's z range; the cable section is positive and the exit lies on the notch side of its connector."""
        manifest = {c['id']: c for c in json.loads((tools.PCB_DIR.parent / 'cad' / 'component-manifest.json').read_text())}
        for ref, motor in (('J4', 'motor_left'), ('J5', 'motor_right')):
            x0, y0, z0, x1, y1, z1 = manifest[motor]['bounds']
            eu, ev, ez = B.MOTOR_CABLE_EXIT[ref]
            face_y = y0 if abs(y0) < abs(y1) else y1
            self.assertAlmostEqual(eu, (x0 + x1) / 2 - B.BOARD_X0, delta=0.05, msg=ref)
            self.assertAlmostEqual(ev, B.BOARD_Y0 - face_y, delta=0.05, msg=ref)
            self.assertTrue(z0 < B.Z_PCB_TOP + ez < z1, ref)
            self.assertEqual(B.EDGE_CONNECTORS[ref], '-v' if ev < 50 else '+v', ref)
        self.assertGreater(B.MOTOR_CABLE_W, 0); self.assertGreater(B.MOTOR_CABLE_T, 0)
    def test_json_gpio_mentions_follow_pico_gpio(self):
        """B3 fix 1: parts-and-connections.json agrees with PICO_GPIO everywhere it names a Pico GPIO. The pico_pins table lists exactly
        the PICO_GPIO pins, each row's GP numbers paired in order with the net names in the last parentheses of its description; every
        other "GPn" in the file is written "GPn (NET)" with PICO_GPIO[NET] == "GPn"."""
        import re
        d = json.loads((tools.PCB_DIR.parent / 'parts-and-connections.json').read_text())
        gp_of = dict(B.PICO_GPIO)
        seen = set()
        for row in d['pico_pins']:
            gps = re.findall(r'\bGP\d+\b', row[0])
            if not gps: continue
            groups = re.findall(r'\(([^()]*)\)', row[1])
            self.assertTrue(groups, row)
            nets = [n.strip() for n in groups[-1].split('/')]
            self.assertEqual(len(nets), len(gps), row)
            for gp, net in zip(gps, nets):
                self.assertEqual(gp_of.get(net), gp, f'pico_pins: {gp} is not {net} in PICO_GPIO')
                seen.add(gp)
        self.assertEqual(seen, set(gp_of.values()))
        rest = {k: v for k, v in d.items() if k != 'pico_pins'}
        def strings(x):
            if isinstance(x, dict):
                for v in x.values(): yield from strings(v)
            elif isinstance(x, list):
                for v in x: yield from strings(v)
            elif isinstance(x, str): yield x
        for text in strings(rest):
            for m in re.finditer(r'\bGP(\d+)\b(\s*\(([A-Z0-9_]+)\))?', text):
                self.assertTrue(m.group(3), f'"GP{m.group(1)}" without its net in: {text[:90]}')
                self.assertEqual(gp_of.get(m.group(3)), f'GP{m.group(1)}', f'GP{m.group(1)} ({m.group(3)}) contradicts PICO_GPIO')
    def test_dnp_spare_rows(self):
        """Ruling PB8: J14 / J15 are DNP 2.54 mm spare pad rows - placed and netted (NETS unchanged), no header fitted, silk "I2C" (ruling
        PB13: J14's shortest label that places in the tongue between H3 / H4; build_pcb puts "DNP" on F.Fab for every DNP part) /
        "SPARE GPIO (DNP)"; a bare header fitted later is BARE_HEADER_H (8.5 mm) tall, the height the placement checks for them."""
        self.assertEqual(B.BARE_HEADER_H, 8.5)
        for ref, n, text in (('J14', 4, 'I2C'), ('J15', 6, 'SPARE GPIO (DNP)')):
            p = B.PARTS[ref]
            self.assertTrue(p.get('dnp'), ref)
            self.assertEqual(p['fp'], f'Connector_PinHeader_2.54mm:PinHeader_1x{n:02d}_P2.54mm_Vertical', ref)
            self.assertEqual(p['h'], B.BARE_HEADER_H, ref)
            self.assertEqual(B.silk_label(ref), f'{ref} {text}')
            self.assertEqual(len([pin for pins in B.NETS.values() for pin in pins if pin[0] == ref]), n, ref)
            self.assertNotIn(ref, B.EDGE_CONNECTORS); self.assertNotIn(ref, B.MATED_H)
    def test_holes_in_placement_match_the_outline(self):
        """PLACEMENT holds H1-H4 at board_outline holes_uv and the clamp holes H5 / H6 at clamp_holes_uv (rotation 0); H5 / H6 are the
        Ø3.0 NPTH holes of ruling PB3 (no pad copper, no net)."""
        o = json.loads((tools.PCB_DIR / 'board_outline.json').read_text())
        for i, (u, v) in enumerate(o['holes_uv'] + o['clamp_holes_uv'], 1): self.assertEqual(B.PLACEMENT[f'H{i}'], (u, v, 0), f'H{i}')
        for r in ('H5', 'H6'):
            self.assertEqual(B.PARTS[r]['fp'], 'MountingHole:MountingHole_3mm', r)
            self.assertEqual([p for pins in B.NETS.values() for p in pins if p[0] == r], [], r)
    def test_tht_pad_gap_rule(self):
        """Spec section 3 / ruling LP9: SMD pads keep SMD_TALL_GAP (2 mm) from every through-hole pad except the M3 holes' pads (ktest_pcb
        measures it); SMD parts fit under a plugged module."""
        self.assertEqual(B.THT_GAP_EXEMPT, ['H1', 'H2', 'H3', 'H4'])
        self.assertFalse(hasattr(B, 'TALL_THT'))
        for r in B.THT_GAP_EXEMPT: self.assertEqual(B.PARTS[r]['fp'], B.FP['HOLE'], r)
        self.assertEqual((B.SMD_GAP, B.SMD_TALL_GAP), (1.0, 2.0))
        self.assertLess(max(p['h'] for p in B.PARTS.values() if p['mount'] == 'smd_top'), B.MODULE_CLEARANCE)
    def test_silk_label_data(self):
        """Fix 1: every part but the holes has a silk label; SMD parts, K1, C1 and the fuse holders show their value, connectors /
        terminals / sockets a short function word; the cliff headers keep the CAD numbering (cliff_1..4 = J6..J9)."""
        for ref in B.PARTS:
            lab = B.silk_label(ref)
            if ref in ('H1', 'H2', 'H3', 'H4', 'H5', 'H6'): self.assertIsNone(lab); continue
            self.assertTrue(lab.startswith(ref + ' '), ref)
            if B.PARTS[ref]['mount'] == 'smd_top' or ref in ('K1', 'C1', 'F1', 'F2'): self.assertEqual(lab, f"{ref} {B.PARTS[ref]['value']}")
            else: self.assertIn(ref, B.SILK_FUNCTION)
        self.assertEqual([B.SILK_FUNCTION[f'J{6 + i}'] for i in range(4)], ['CLIFF1', 'CLIFF2', 'CLIFF3', 'CLIFF4'])
        self.assertEqual([B.PARTS[f'J{6 + i}']['value'] for i in range(4)], ['Cliff 1', 'Cliff 2', 'Cliff 3', 'Cliff 4'])
    def test_power_zones_are_the_trunks_on_their_layers(self):
        """Bay B4 (4 layers): POWER_ZONES entries are dict(net, layers, poly[, trunk]) on F.Cu / In2.Cu / B.Cu (In1.Cu is the solid GND
        plane and the B.Cu GND fill is build_pcb's own, so no entry is GND). A (net, layer) has one polygon at most. Every POWER_TRUNK net
        but GND - BAT+, F1_OUT, F2_OUT, VBAT_SW, VMOT, 5V - has at least one trunk layer (no trunk net is zoneless), and a trunk polygon
        contains the placed centre of every POWER_TRUNK pad of its net. trunk=False marks a helper pour (In2 copper that does not join
        all of the net's trunk pads; B5 stitches it with vias) - not a trunk. 5V's trunk includes the B.Cu strip U5B -> J3."""
        seen = set()
        for z in B.POWER_ZONES:
            self.assertLessEqual(set(z), {'net', 'layers', 'poly', 'trunk', 'spoke'}, z)
            self.assertTrue(1.2 <= z.get('spoke', 1.2) <= 2.0, z['net'])    # R13 / PB12 relief spokes
            net, layers, pts = z['net'], z['layers'], z['poly']
            self.assertIn(net, B.NETS); self.assertNotEqual(net, 'GND')
            self.assertIsInstance(layers, tuple)
            self.assertTrue(layers and set(layers) <= {'F.Cu', 'In2.Cu', 'B.Cu'}, (net, layers))
            self.assertEqual(list(layers), [L for L in C.STACKUP if L in layers], (net, layers))
            for L in layers:
                self.assertNotIn((net, L), seen, f'{net} has two polygons on {L}'); seen.add((net, L))
            self.assertGreaterEqual(len(pts), 4, net)
            for u, v in pts: self.assertTrue(0 <= u <= B.BOARD_W and 0 <= v <= B.BOARD_H, (net, u, v))
            if z.get('trunk', True):
                self.assertIn(net, B.POWER_TRUNK, f'{net}: a trunk zone of a net without POWER_TRUNK pads')
                for ref, pin in B.POWER_TRUNK[net]:
                    self.assertTrue(point_in_polygon(B.pad_uv(ref, pin), pts), f'{net} {layers}: {ref}.{pin} at {B.pad_uv(ref, pin)} outside its zone')
        self.assertEqual(set(B.POWER_TRUNK) - {'GND'}, {'BAT+', 'F1_OUT', 'F2_OUT', 'VBAT_SW', 'VMOT', '5V'})
        self.assertEqual([n for n in B.POWER_TRUNK if n != 'GND' and not B.zone_layers(n)], [], 'zoneless trunk nets')
        for net in B.POWER_TRUNK:
            if net != 'GND':
                self.assertEqual(B.zone_layers(net), tuple(L for L in C.STACKUP if (net, L) in seen and any(
                    z['net'] == net and L in z['layers'] and z.get('trunk', True) for z in B.POWER_ZONES)), net)
        self.assertIn('B.Cu', B.zone_layers('5V'))                           # controller hand-over: the 5V trunk on 1 oz B.Cu
        self.assertLessEqual({'VBAT_SW', 'VMOT', '5V', '3V3'}, {z['net'] for z in B.POWER_ZONES if 'In2.Cu' in z['layers']})

    def test_pad_current_per_trunk_pin(self):
        """PB12 (B4 fix 1): every POWER_TRUNK pin has its own load, board_spec.pad_current (PAD_CURRENT_A, else CURRENT_A[net]): the battery
        path (J1, J2, F1) 7.5 A, the motor fuse and relay (F2, K1) 5.0 A, the Jetson lead (J3) 4.5 A, the P03 input 5 V x 4.5 A / (6.0 V x
        0.9) = 4.17 A and its output 4.5 A, each driver's VIN / GND pin and C1 2.1 A."""
        pins = [(r, p) for pads in B.POWER_TRUNK.values() for r, p in pads]
        self.assertLessEqual(set(B.PAD_CURRENT_A), set(pins))
        want = {**{(r, p): 7.5 for r in ('J1', 'J2', 'F1') for p in ('1', '2')}, **{(r, p): 5.0 for r in ('F2',) for p in ('1', '2')},
                ('K1', B.RELAY_PINS['COM']): 5.0, ('K1', B.RELAY_PINS['NO']): 5.0, ('J3', '1'): 4.5, ('J3', '2'): 4.5,
                ('U5B', '1'): 4.5, ('U5B', '2'): 4.5, ('C1', '1'): 2.1, ('C1', '2'): 2.1,
                **{(r, p): 2.1 for r in ('U3P', 'U4P') for p in ('1', '2')}}
        for k, a in want.items(): self.assertAlmostEqual(B.pad_current(*k), a, 6, k)
        self.assertAlmostEqual(B.P03_IN_A, 5.0 * 4.5 / (6.0 * 0.9), 6)
        for k in (('U5A', '1'), ('U5A', '2')): self.assertAlmostEqual(B.pad_current(*k), B.P03_IN_A, 6, k)
        for r, p in pins: self.assertGreater(B.pad_current(r, p), 0, (r, p))
    def test_tap_loads_cover_every_tap_pad(self):
        """B6 carry item 4: every pad of a tap net (TAP_NETS: the supplies with tracks to small loads) that is not a trunk pin and not
        the net's source pin has its own load in TAP_LOAD_A (A, >= 0; probe pads 0), so ktest_power can sum the load behind each
        track. The 3V3 source is the Pico's 3V3 pin; the zone nets are fed by their trunk zones. The 3V3 loads stay within the Pico's
        300 mA, the 5V taps within the spec's ~0.3 A beside the Jetson."""
        self.assertEqual(B.TAP_NETS, ('VBAT_SW', 'VMOT', '5V', '3V3'))
        self.assertEqual(B.TAP_SOURCE, {'3V3': B.pico('3V3_OUT')})
        trunk = {p for pins in B.POWER_TRUNK.values() for p in pins}
        want = {p for n in B.TAP_NETS for p in B.NETS[n] if p not in trunk and p != B.TAP_SOURCE.get(n)}
        self.assertEqual(set(B.TAP_LOAD_A), want)
        for p, a in B.TAP_LOAD_A.items(): self.assertGreaterEqual(a, 0.0, p)
        for i in range(1, 6): self.assertEqual(B.TAP_LOAD_A.get((f'TP{i}', '1'), 0.0), 0.0)
        total = lambda net: sum(a for p, a in B.TAP_LOAD_A.items() if p in B.NETS[net])
        self.assertLessEqual(total('3V3'), 0.300)
        self.assertLessEqual(total('5V'), 0.5)

    def test_c3_ripple_and_height_data(self):
        """B6 carry item 6 / ruling PB20 (d): C3's datasheet ripple rating, its ESR and C1's, the DRV8874 PWM frequency assumed, and C3's
        height tolerance. The 6.3 x 7.7 mm can (7.7 + 0.3 mm) met MODULE_CLEARANCE (8.0 mm) with no margin, so PB20 takes the 6.3 x 5.8 mm
        can: worst-case height <= C3_H_MAX (7.5 mm), >= 68 uF, rated >= 1.5 x the highest battery voltage (VBAT_MAX)."""
        self.assertGreater(B.C3_RIPPLE_RATING_A, 0); self.assertGreater(B.C3_ESR_OHM, 0); self.assertGreater(B.C1_ESR_OHM, 0)
        self.assertGreater(B.DRV_PWM_HZ, 0)
        self.assertEqual(B.PARTS['C3']['fp'], 'Capacitor_SMD:CP_Elec_6.3x5.8')
        self.assertEqual(B.C3_H_MAX, 7.5)
        self.assertLessEqual(B.PARTS['C3']['h'] + B.PART_H_TOL['C3'], B.C3_H_MAX)
        self.assertLessEqual(B.C3_H_MAX, B.MODULE_CLEARANCE - 0.5)
        uf, volts = B.PARTS['C3']['value'].split()
        self.assertGreaterEqual(float(uf.rstrip('uF')), 68.0)
        self.assertGreaterEqual(float(volts.rstrip('V')), 1.5 * B.VBAT_MAX)
        self.assertIn('EEE-FK1C101P', B.PARTS['C3']['mpn'])

    def test_j1_pinout_pb20(self):
        """Ruling PB20 (a): J1's pins swapped - pin 1 (the south pin, by the free F.Cu west strip, K1.5 and the main B.Cu GND fill) is
        GND, pin 2 (north, toward J2 / F1) is BAT+; the silk carries '+' at the BAT+ pin and '-' at the GND pin (PIN_MARKS)."""
        self.assertEqual(B.J1_PINOUT, {'1': 'GND', '2': 'BAT+'})
        self.assertEqual((B.J1_GND_PIN, B.J1_BAT_PIN), (('J1', '1'), ('J1', '2')))
        self.assertIn(('J1', '1'), B.NETS['GND']); self.assertIn(('J1', '2'), B.NETS['BAT+'])
        self.assertNotIn(('J1', '1'), B.NETS['BAT+']); self.assertNotIn(('J1', '2'), B.NETS['GND'])
        self.assertEqual(B.POWER_TRUNK['BAT+'], [('J1', '2'), ('F1', '1')])
        self.assertIn(('J1', '1'), B.POWER_TRUNK['GND']); self.assertNotIn(('J1', '2'), B.POWER_TRUNK['GND'])
        self.assertEqual(B.PIN_MARKS, {'J1': {'2': '+', '1': '-'}})
        for ref, marks in B.PIN_MARKS.items():
            self.assertEqual(set(B.PIN_MARK_AT[ref]), set(marks))         # B6 fix 1: one offset per marked pin
            for pin, sign in marks.items():
                self.assertIn((ref, pin), B.NETS['BAT+' if sign == '+' else 'GND'])
                # outside the housing (ktest_pcb checks the body): each mark lies past its own pin's end of the row, away from the other
                other = next(q for q in marks if q != pin)
                dv = B.pad_uv(ref, pin)[1] - B.pad_uv(ref, other)[1]
                self.assertGreater(B.PIN_MARK_AT[ref][pin][1] * dv, 0, (ref, pin))

    def test_gnd_patch_pins_pb21(self):
        """Ruling PB21 (B6 fix 2): one pin per F.Cu GND patch (GND_PATCH_PINS, GND trunk pins), the only pad with its own spoke angle is
        U3P.2 (90 degrees, toward its patch), the only one-spoke DRC exception names that pad, and nothing is pending."""
        self.assertEqual(len(B.GND_PATCH_PINS), len(B.GND_PATCHES))
        for pin in B.GND_PATCH_PINS: self.assertIn(pin, B.POWER_TRUNK['GND'])
        self.assertEqual(B.PAD_SPOKE_ANGLE, {('U3P', '2'): 90.0})
        self.assertLessEqual(set(B.PAD_SPOKE_ANGLE), set(B.GND_PATCH_PINS))
        self.assertEqual(B.DRC_RULES.count('min_resolved_spokes'), 1)
        self.assertEqual((B.PAD_PENDING, B.STITCH_PENDING), ({}, {}))

    def test_stitch_vias_are_listed_pre_route_vias(self):
        """B4 fix 1: the stitching vias (GND patches / B.Cu GND islands to the In1 plane, the F1_OUT In2 copy to its B.Cu trunk; BAT+'s went with PB20)
        are (net, (u, v)) pre-route vias inside the board box, on power or GND nets, of their class's size (net_via), never two closer than
        their radii + 0.25 mm."""
        self.assertTrue(B.STITCH_VIAS)
        allv = [(B.net_via(n)[0], uv) for n, uv in B.STITCH_VIAS] + [(B.net_via('GND')[0], uv) for uv in B.SMD_GND_VIAS.values()]
        for net, (u, v) in B.STITCH_VIAS:
            self.assertIn(net, ('GND', 'BAT+', 'F1_OUT')); self.assertTrue(0 < u < B.BOARD_W and 0 < v < B.BOARD_H, (net, u, v))
        self.assertEqual((B.net_via('GND'), B.net_via('BAT+'), B.net_via('F1_OUT')), ((0.8, 0.4), (1.2, 0.6), (1.2, 0.6)))
        for i, (da, a) in enumerate(allv):
            for dc, c in allv[i + 1:]: self.assertGreaterEqual(((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2) ** 0.5, (da + dc) / 2 + 0.25, (a, c))

def point_in_polygon(p, pts):
    x, y = p; inside = False
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1): inside = not inside
    return inside

def module_misalignments(placement):
    """[(module, socket, expected (u, v, rot), placed)] for sockets that do not sit on the module's real pin rows."""
    bad = []
    for name, m in B.MODULES.items():
        first, (fx, fy) = next(iter(m['rows'].items()))
        u0, v0, r0 = placement[first]
        for ref, (x, y) in m['rows'].items():
            eu, ev = B.local_to_board((u0, v0, r0), (x - fx, y - fy))
            u, v, r = placement[ref]
            if abs(u - eu) > 1e-6 or abs(v - ev) > 1e-6 or (r - r0) % 360:
                bad.append((name, ref, (round(eu, 3), round(ev, 3), r0), (u, v, r)))
    return bad

if __name__ == '__main__':
    unittest.main()
