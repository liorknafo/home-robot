"""R1 power & I/O board — single source of truth (spec 2026-10-04-r1-power-io-board-design.md, sections 3-7).
Pure Python (no numpy): imported by the CAD venv and by KiCad's bundled Python."""
import math
from functools import lru_cache
from pathlib import Path
from pcb.sexpr import parse, find
from pcb import tools, copper

# Board frame: u = robot x - BOARD_X0 (toward the robot front), v = BOARD_Y0 - robot y (toward robot right).
# Bay board (spec 2026-10-08-r1-bay-board, B2): one 4-layer 1.6 mm PCB at base level under the Jetson, robot x -66.5..7.5, y -50..50 (74 x 100 mm,
# <= 100 x 100 for the JLCPCB price tier). Its bottom is z 35.4 (P_base top z 33 + 2.4 mm printed PETG spacers), its top z 37.0. The real outline is
# cut from the CAD by extract_outline.py (>= 1.5 mm to every fixed part): the battery tray wall behind it (x -68), the P03 regulator in front (x 9),
# the two motors beside the tongue (|y| >= 35).
BOARD_X0, BOARD_Y0, BOARD_W, BOARD_H = -66.5, 50.0, 74.0, 100.0
Z_PCB_BOTTOM, PCB_T = 35.4, 1.6
Z_PCB_TOP = round(Z_PCB_BOTTOM + PCB_T, 3)
# Part-height limits above the board top (component body + socket, as PARTS[..]['h']).
# Behind the Jetson carrier (x < -38.5) the lid is the ceiling: parts stay <= lid underside - 3 mm (z 97.5), 60.5 mm over the board top.
# Under the carrier (x >= -38.5, the carrier's rear edge x -37 less 1.5 mm) every part top - a mated plug and its wire bend included - stays
# CARRIER_CLEARANCE under the carrier's lowest lead (z 54.77, the measured 55.07 less 0.3 mm): 16.27 mm over the board top. There is no top
# access once the Jetson is fitted. The old carrier-band rule (BAND_PLAN_MARGIN / BAND_PLAN_EXEMPT) is retired with the strip.
LID_UNDERSIDE_Z = 100.5
CEILING_Z_LID = round(LID_UNDERSIDE_Z - 3.0, 3)
MAX_PART_H = round(CEILING_Z_LID - Z_PCB_TOP, 3)
CARRIER_X = -38.5
CARRIER_U = round(CARRIER_X - BOARD_X0, 3)
UNDER_CARRIER_Z = 54.77
CARRIER_CLEARANCE = 1.5
UNDER_CARRIER_MAX_H = round(UNDER_CARRIER_Z - CARRIER_CLEARANCE - Z_PCB_TOP, 3)
# (u0, v0, u1, v1, max_h): boxes of the board where parts must be lower than MAX_PART_H. A zone that reaches a board edge (here u1 = BOARD_W and
# both side edges) continues past it: the carrier does too.
HEIGHT_ZONES = [(CARRIER_U, 0.0, BOARD_W, BOARD_H, UNDER_CARRIER_MAX_H)]
def max_part_h_at(u, v):
    """Tallest part (mm above the board top) allowed at board point (u, v): the limit of the zone containing it (zone edges belong to the zone), else MAX_PART_H."""
    return min([MAX_PART_H] + [h for u0, v0, u1, v1, h in HEIGHT_ZONES if u0 <= u <= u1 and v0 <= v <= v1])
CUSTOM_FP_DIR = Path(__file__).parent / 'r1.pretty'
def robot_to_board(x, y): return (x - BOARD_X0, BOARD_Y0 - y)
def board_to_robot(u, v): return (u + BOARD_X0, BOARD_Y0 - v)

# --- Vendor pinouts (Task 2 Step 4; sources and condensed notes in pcb/reference/) ---
# Pololu 4035 (DRV8874 carrier): two rows of 7 holes, 2.54 mm pitch, rows 12.70 mm apart, pin 1 of both rows at the same end.
# Row 'L' = left row of Pololu's top view (component side up, pin 1 up): 1 VM, 2 GND, 3 EN/IN1, 4 PH/IN2, 5 PMODE, 6 SLEEP, 7 VREF.
# Row 'P' = right row of the same view:                                   1 VIN, 2 GND, 3 OUT1, 4 OUT2, 5 IMODE, 6 FAULT, 7 CS.
# (Names 'logic row' / 'power row' are only labels: the status pins IMODE/FAULT/CS sit on the VIN row.)
DRV_ROWS = {'L': 7, 'P': 7}        # pins in the carrier's logic row and power row (from pololu-4035.txt)
DRV_ROW_SPACING = 12.70            # mm between the two socket rows; PLACEMENT of U3L/U3P and U4L/U4P must honour this (Task 5)
DRV_PINS = {'VIN': ('P', 1), 'GND': ('P', 2), 'OUT1': ('P', 3), 'OUT2': ('P', 4), 'IN1': ('L', 3), 'IN2': ('L', 4),
            'PMODE': ('L', 5), 'SLEEP': ('L', 6), 'FAULT': ('P', 6), 'CS': ('P', 7)}   # name -> (row 'L'|'P', pin number 1..7)
DRV_PMODE_NET = '3V3'              # PMODE high = PWM (IN/IN) control, latched when SLEEP goes high (pololu-4035.txt)
DRV_EXTRA_GND = [('L', 2)]         # the carrier's second GND pin, tied to GND
DRV_UNUSED = [('L', 1), ('P', 5), ('L', 7)]   # VM (carrier's protected-supply output), IMODE (carrier pulls it to GND), VREF (carrier ties it to SLEEP)
# Pololu 2851 (D24V50F5): one row of 5 holes, 2.54 mm pitch: EN, VIN, GND, GND, VOUT (EN has an on-board pull-up to VIN: left NC).
REG_B_PINS = {'EN': 1, 'VIN': 2, 'GND': 3, 'GND2': 4, 'VOUT': 5}   # name -> pin number (pololu-2851.txt)
# Vendor record only: the D24V50F5 socket (U5C) is not fitted on the strip board (L5); the user's P03 on U5A / U5B is the regulator.
RELAY_SYM = 'Relay:G5LE-1'         # Omron G5LE-1 (pin map from the Omron datasheet, omron-g5le-1.txt)
RELAY_PINS = {'COIL_P': '2', 'COIL_N': '5', 'COM': '1', 'NO': '3', 'NC': '4'}   # coil is unpolarised: P/N only places the flyback diode
# KiCad 9's stock Fuse library has a real, buyable through-hole ATO holder (Littelfuse FLR 178.6165.0001, 30 A, 80 V; 4 solder pins
# per contact, so pads are numbered 1,1,1,1,2,2,2,2): no custom library.
FUSE_FP = 'Fuse:FuseHolder_Blade_ATO_Littelfuse_FLR_178.6165'
FUSE_MPN = 'Littelfuse 178.6165.0001'
# Q1-Q5 are 2N7002 in SOT-23. KiCad's Transistor_FET:2N7002 (extends Q_NMOS_GSD) numbers G=1, S=2, D=3, where the 2N7000 TO-92 it replaces
# was S=1, G=2, D=3; Package_TO_SOT_SMD:SOT-23 has pads 1, 2, 3. NETS name the pin by role through q(), so the remap lives in this one table
# (test_transistor_and_diode_remap_by_symbol_pin_name checks it against the symbol's pin names).
Q_SYM = 'Transistor_FET:2N7002'
Q_PINS = {'G': '1', 'S': '2', 'D': '3'}
def q(i, role): return (f'Q{i}', Q_PINS[role])
# D1 / D2 are SS14 (SMA). Diode:SS14 and Device:D_Schottky both number K=1, A=2; Diode_SMD:D_SMA pad 1 is the cathode (band side).
D_SYM = 'Diode:SS14'
D_PINS = {'K': '1', 'A': '2'}
def dio(ref, role): return (ref, D_PINS[role])

# --- Pico (two 1x20 sockets): physical pin -> socket pin ---
PICO_PIN = {'GP0': 1, 'GP1': 2, 'GND1': 3, 'GP2': 4, 'GP3': 5, 'GP4': 6, 'GP5': 7, 'GND2': 8, 'GP6': 9, 'GP7': 10,
            'GP8': 11, 'GP9': 12, 'GND3': 13, 'GP10': 14, 'GP11': 15, 'GP12': 16, 'GP13': 17, 'GND4': 18, 'GP14': 19,
            'GP15': 20, 'GP16': 21, 'GP17': 22, 'GND5': 23, 'GP18': 24, 'GP19': 25, 'GP20': 26, 'GP21': 27, 'GND6': 28,
            'GP22': 29, 'RUN': 30, 'GP26': 31, 'GP27': 32, 'AGND': 33, 'GP28': 34, 'ADC_VREF': 35, '3V3_OUT': 36,
            '3V3_EN': 37, 'GND7': 38, 'VSYS': 39, 'VBUS': 40}
def pico(name):
    n = PICO_PIN[name]
    return ('U1A', str(n)) if n <= 20 else ('U1B', str(41 - n))     # right row runs bottom-to-top
GY521 = ['VCC', 'GND', 'SCL', 'SDA', 'XDA', 'XCL', 'AD0', 'INT']    # U2 socket pins 1..8
def imu(name): return ('U2', str(GY521.index(name) + 1))
MOTOR = ['M+', 'M-', 'ENC_GND', 'ENC_VCC', 'A', 'B']               # Pololu motor cable order, J4/J5 pins 1..6
def drv(side, name):
    row, pin = DRV_PINS[name]
    return (f'U{3 if side == "L" else 4}{row}', str(pin))

FP = dict(VH2='Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical',
          # B12a (ruling PB30): J1 / J2 / J3 are three connector families that cannot be cross-plugged - the battery an XT30 (vertical PCB
          # male; the right-angle XT30PW-M has no cable path at the rear edge, see PARTS), the rocker a JST VH, the Jetson 5 V a Molex
          # Micro-Fit 3.0 right-angle header.
          XT30V='Connector_AMASS:AMASS_XT30UPB-M_1x02_P5.0mm_Vertical',
          MF2R='Connector_Molex:Molex_Micro-Fit_3.0_43650-0200_1x02_P3.00mm_Horizontal',
          XH2='Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical',
          XH4='Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical', XH5='Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical',
          HDR6='Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical', HDR4='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical',
          HDR6R='Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Horizontal',      # B3: right-angle, the C18 pigtail socket lies on it (PB25)
          # Small parts are surface-mount, top side only, standard (not HandSolder) footprints, reflowed with paste and a stencil (spec
          # 2026-10-06-r1-low-profile section 3, ruling LP4). Everything else below stays through-hole.
          R='Resistor_SMD:R_0805_2012Metric', C='Capacitor_SMD:C_0805_2012Metric', Q='Package_TO_SOT_SMD:SOT-23',
          D='Diode_SMD:D_SMA', LED='LED_SMD:LED_0805_2012Metric', TP='TestPoint:TestPoint_Pad_D2.0mm',
          CP='Capacitor_THT:CP_Radial_D10.0mm_P5.00mm', CPV='Capacitor_SMD:CP_Elec_6.3x5.8', HOLE='MountingHole:MountingHole_3.2mm_M3_Pad_Via',
          TERM2='TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal',
          CLAMP='MountingHole:MountingHole_3mm')           # B3 / ruling PB3: the rear Jetson standoff clamp holes, Ø3.0 NPTH
def sock(n): return f'Connector_PinSocket_2.54mm:PinSocket_1x{n:02d}_P2.54mm_Vertical'
def conn(n): return f'Connector_Generic:Conn_01x{n:02d}'

# mount: 'tht' (hand-soldered after the reflow) or 'smd_top' (paste + reflow; top side only, so the board can sit on a hot plate). The assembly
# spacing rules (>= 1 mm SMD to SMD, >= 2 mm SMD pad to the pads of tall THT parts) key on it.
def P(sym, fp, value, mpn='', h=10.0, mount='tht'): return dict(sym=sym, fp=fp, value=value, mpn=mpn, h=h, mount=mount)
def SMD(sym, fp, value, mpn='', h=1.0): return P(sym, fp, value, mpn, h, 'smd_top')
def DNP(part):
    """Do-not-populate (ruling PB8): the footprint is placed and netted, the part is not fitted; the BOM lists it as optional."""
    part['dnp'] = True
    return part
def NOBOM(part):
    """Not a bought part (bay B7): a bare copper test pad or a mounting / clamp hole. Its footprint says exclude_from_bom, and the schematic
    symbol follows (make_schematic: in_bom no), so the schematic parity holds; the BOM lists it as 'no part'."""
    part['bom'] = False
    return part
def in_bom(ref): return PARTS[ref].get('bom', True)
# Bare 2.54 mm pin header fitted later on a DNP spare row (2.54 mm plastic + 6 mm pins, rounded up): the height it must still clear.
BARE_HEADER_H = 8.5
# Manufacturer part numbers (bay B7 BOM; B9 owns the BOM check): the ones quoted from memory say "verify before ordering".
HDR6R_MPN = 'Amphenol 68016-106HLF (1x6 2.54 mm right-angle pin header; verify before ordering)'
SOCKET_MPN = {n: f'Sullins PPTC{n:02d}1LFBN-RC (1x{n} 2.54 mm female socket, 8.5 mm body; verify before ordering)' for n in (7, 8, 20)}
TERM2_MPN = 'Phoenix Contact 1715721 (MKDS 1,5/ 2-5,08; verify before ordering)'
LED_MPN = {'green': 'Lite-On LTST-C171GKT (green 0805; verify before ordering)', 'yellow': 'Lite-On LTST-C171YKT (yellow 0805; verify before ordering)'}
# B12a (ruling PB30): J1 / J2 / J3 sat side by side in the band as three identical JST VH-2 headers, so each lead fitted each header -
# the battery lead into J3 feeds the pack backwards and unfused into the 5 V net, the rocker pair into J1 shorts the pack through the rocker
# ahead of F1, the Jetson lead into J2 puts pack voltage on the Jetson's 5 V input. Now three families (connector_family below), none of
# whose plugs fits another's header, all polarised:
#  - J1 battery: AMASS XT30UPB-M, the vertical PCB male (XT30, 15 A continuous; mates the XT30U-F on the battery lead W01 - the female, shrouded
#    half on the source side). The controller's XT30PW-M (right-angle) has a horizontal mating axis: facing the rear edge its mated XT30U-F and
#    wire bend would run into the battery tray's front wall 1.5 mm behind the edge (and the battery, x -71), and facing along the edge or
#    forward it needs a ~25-30 mm free lane at <= 6 mm over the board that the band does not have (J12 / J2 / K1 on the edge, the 30 mm fuse
#    holders ahead). The vertical part keeps today's cable path (plugged from above with the lid off, the lead up and over the tray wall).
#    Pin 1 is the footprint's '-' pad (GND), pin 2 its '+' pad (BAT+): J1_PINOUT unchanged (PB20).
#  - J3 Jetson 5 V: Molex 43650-0200, the Micro-Fit 3.0 1x2 right-angle through-hole header (3 mm locating peg; -0210 is the SMD-tail
#    variant), mating face at the right edge; W04 ends in a 43645-0200 receptacle with 43030 crimps (18 AWG). Micro-Fit 3.0: 8.5 A per
#    circuit for 2 circuits on 18 AWG (Molex PS-43045 derating table; the older catalogue figure is 5 A) against 4.5 A: verify.
#  - J2 rocker: JST VH (B2P-VH, unchanged).
XT30_MPN = 'AMASS XT30UPB-M (XT30 vertical PCB male, 2-pin, 15 A continuous; LCSC C428721; verify before ordering)'
MF2R_MPN = 'Molex 43650-0200 (Micro-Fit 3.0 1x2 right-angle THT header with peg, 8.5 A/circuit on 18 AWG; verify before ordering)'
PARTS = {
    # J1 h 11: the XT30UPB-M housing over the board (verify on the AMASS drawing); J3 h 10: the Micro-Fit right-angle housing (verify)
    'J1': P(conn(2), FP['XT30V'], 'Battery in', XT30_MPN, h=11.0), 'J2': P(conn(2), FP['VH2'], 'Master rocker', 'JST B2P-VH(LF)(SN)'),
    'J3': P(conn(2), FP['MF2R'], 'Jetson 5V', MF2R_MPN, h=10.0),
    # J4 / J5 (B3): right-angle 0.1 in headers; the C18 pigtails' right-angle 1x6 sockets lie on the board under the carrier, the motors'
    # factory cables mate at the inline joints off the board (PB25)
    'J4': P(conn(6), FP['HDR6R'], 'Left motor', HDR6R_MPN, h=2.54), 'J5': P(conn(6), FP['HDR6R'], 'Right motor', HDR6R_MPN, h=2.54),
    **{f'J{6 + i}': P(conn(5), FP['XH5'], f'Cliff {i + 1}', 'JST B5B-XH-A(LF)(SN)') for i in range(4)},
    'J10': P(conn(2), FP['XH2'], 'Bumper L', 'JST B2B-XH-A(LF)(SN)'), 'J11': P(conn(2), FP['XH2'], 'Bumper R', 'JST B2B-XH-A(LF)(SN)'),
    'J12': P(conn(2), FP['XH2'], 'Arm button', 'JST B2B-XH-A(LF)(SN)'), 'J13': P(conn(2), FP['XH2'], 'Motor toggle loop', 'JST B2B-XH-A(LF)(SN)'),
    # PB8: the spares are DNP 2.54 mm pad rows (no header fitted, no edge, no cable lane); a bare header fitted later is BARE_HEADER_H tall
    'J14': DNP(P(conn(4), FP['HDR4'], 'Spare I2C', h=BARE_HEADER_H)), 'J15': DNP(P(conn(6), FP['HDR6'], 'Spare GPIO', h=BARE_HEADER_H)),
    # Pico h 15: socket 8.5 + header spacer 2.5 + PCB 1.0 + micro-USB receptacle about 2.9 (L5 fix 1, was 12)
    'U1A': P(conn(20), sock(20), 'Pico left row', SOCKET_MPN[20], h=15), 'U1B': P(conn(20), sock(20), 'Pico right row', SOCKET_MPN[20], h=15),
    'U2': P(conn(8), sock(8), 'GY-521 MPU-6050', SOCKET_MPN[8], h=14),
    # the sockets are the parts; the Pololu 4035 carriers, the Pico and the GY-521 that plug into them are BOM_EXTRAS rows
    'U3L': P(conn(DRV_ROWS['L']), sock(DRV_ROWS['L']), 'DRV8874 left, VM/EN/PH row', SOCKET_MPN[7], 14),
    'U3P': P(conn(DRV_ROWS['P']), sock(DRV_ROWS['P']), 'DRV8874 left, VIN/OUT row', SOCKET_MPN[7], 14),
    'U4L': P(conn(DRV_ROWS['L']), sock(DRV_ROWS['L']), 'DRV8874 right, VM/EN/PH row', SOCKET_MPN[7], 14),
    'U4P': P(conn(DRV_ROWS['P']), sock(DRV_ROWS['P']), 'DRV8874 right, VIN/OUT row', SOCKET_MPN[7], 14),
    'U5A': P(conn(2), FP['TERM2'], 'Regulator in (P03 module)', TERM2_MPN, h=12),
    'U5B': P(conn(2), FP['TERM2'], 'Regulator out (P03 module)', TERM2_MPN, h=12),
    # h 30: holder body 17.5 mm + the ATO fuse body standing about 12 mm above it (littelfuse-flr-178.6165.txt)
    'F1': P('Device:Fuse', FUSE_FP, '7.5 A ATO', FUSE_MPN, 30), 'F2': P('Device:Fuse', FUSE_FP, '5 A ATO', FUSE_MPN, 30),
    'K1': P(RELAY_SYM, 'Relay_THT:Relay_SPDT_Omron-G5LE-1', 'G5LE-1 DC5', 'Omron G5LE-1 DC5', 19),
    'D1': SMD(D_SYM, FP['D'], 'SS14', 'Vishay SS14-E3/61T (SMA 40 V 1 A Schottky; verify before ordering)', 2.7),      # SMA body 2.7 mm max: the tallest SMD part after C3
    # LIO-226: D2, 5V -> the Pico's VSYS, DNP. As delivered the Pico and the whole 3V3 rail run from the Jetson's USB port only (VBUS /
    # VSYS were NC); fitting D2 runs them from the board's 5V too (the Pico's own VBUS diode blocks the back-feed into the USB host).
    'D2': DNP(SMD(D_SYM, FP['D'], 'SS14', 'Vishay SS14-E3/61T (SMA 40 V 1 A Schottky; verify before ordering)', 2.7)),
    'C1': P('Device:C_Polarized', FP['CP'], '470uF 25V', 'Panasonic EEU-FR1E471', 20),
    'C2': SMD('Device:C', FP['C'], '100nF', 'Samsung CL21B104KBCNNNC (100 nF 50 V X7R 0805; verify before ordering)', 1.25),
    # LIO-226: C4 / C5, the 100 nF of the L_CS / R_CS RC filters (with R24 / R25 and the carrier's 2.49 k CS load: (2.49 k + 4.7 k) x
    # 100 nF = 0.72 ms, LIO-234), cap to GND as C2
    **{c: SMD('Device:C', FP['C'], '100nF', 'Samsung CL21B104KBCNNNC (100 nF 50 V X7R 0805; verify before ordering)', 1.25) for c in ('C4', 'C5')},
    # PB16: DRV_L's bulk cap. PB20 (d): the 6.3 x 5.8 mm FK can (5.8 + 0.3 mm worst case, under C3_H_MAX), 16 V >= 1.5 x VBAT_MAX
    'C3': SMD('Device:C_Polarized', FP['CPV'], '100uF 16V', 'Panasonic EEE-FK1C101P (FK, low ESR, 6.3 x 5.8 mm; verify before ordering)', 5.8),
    **{f'Q{i}': SMD(Q_SYM, FP['Q'], '2N7002', 'Nexperia 2N7002,215', 1.2) for i in range(1, 6)},
    **{f'LED{i}': SMD('Device:LED', FP['LED'], c, LED_MPN[c], 1.1) for i, c in ((1, 'green'), (2, 'green'), (3, 'yellow'))},
    **{f'TP{i}': NOBOM(SMD('Connector:TestPoint', FP['TP'], n, h=0.1)) for i, n in enumerate(('VBAT_SW', 'VMOT', '5V', '3V3', 'GND'), 1)},   # flat pads
    **{f'H{i}': NOBOM(P('Mechanical:MountingHole_Pad', FP['HOLE'], 'M3', h=0)) for i in range(1, 5)},
    **{f'H{i}': NOBOM(P('Mechanical:MountingHole', FP['CLAMP'], 'M2.5 clamp', h=0)) for i in (5, 6)},     # PB3: no pad copper, no net
}
RES = {  # ref: (value, net a, net b)
    'R1': ('100k', 'ENC_LA_5V', 'GND'), 'R2': ('10k', 'ENC_LA_3V3', '3V3'), 'R3': ('100k', 'ENC_LB_5V', 'GND'), 'R4': ('10k', 'ENC_LB_3V3', '3V3'),
    'R5': ('100k', 'ENC_RA_5V', 'GND'), 'R6': ('10k', 'ENC_RA_3V3', '3V3'), 'R7': ('100k', 'ENC_RB_5V', 'GND'), 'R8': ('10k', 'ENC_RB_3V3', '3V3'),
    'R9': ('100k', 'COIL_P', 'GND'), 'R10': ('10k', 'MOTOR_ON_N', '3V3'),
    'R11': ('100k', 'VBAT_SW', 'VSENSE'), 'R12': ('33k', 'VSENSE', 'GND'),
    'R13': ('10k', 'L_IN1', 'GND'), 'R14': ('10k', 'L_IN2', 'GND'), 'R15': ('10k', 'R_IN1', 'GND'), 'R16': ('10k', 'R_IN2', 'GND'),
    'R17': ('100k', 'DRV_SLEEP', 'GND'), 'R18': ('10k', 'DRV_FAULT', '3V3'),
    # LIO-226: 10 k (were 4.7 k). With the GY-521's 2.2 k and the four VL53L0X carriers' 10 k on the bus, 4.7 k made 0.94 k in all
    # (3.1 mA at 0.4 V, over I2C's 3 mA); 10 k makes 1.05 k (2.8 mA)
    'R19': ('10k', 'SDA', '3V3'), 'R20': ('10k', 'SCL', '3V3'),
    'R21': ('3.3k', 'VBAT_SW', 'LED1_A'), 'R22': ('1k', '5V', 'LED2_A'), 'R23': ('3.3k', 'VMOT', 'LED3_A'),
    # LIO-226: the DRV8874 CS outputs (1.1 V/A of the high-side current, so I x duty under slow decay (LIO-234); a 20 kHz chopped
    # waveform, about 3.2-3.3 V at the current limit against the ADC pin's IOVDD + 0.3 V) reach GP27 / GP28 through 4.7 k, with C4 / C5
    # (100 nF) to GND on the ADC side
    'R24': ('4.7k', 'L_CS', 'L_CS_ADC'), 'R25': ('4.7k', 'R_CS', 'R_CS_ADC'),
    # LIO-226: ARM (normally-open button on a 200 mm lead) pulled up on the board, not by the RP2040's 50-80 k next to the motor PWM.
    # BUMP_L / BUMP_R (normally closed: an open lead reads as a bump, fail-safe) stay on the internal pull-ups.
    'R26': ('10k', 'ARM', '3V3'),
}
def res_mpn(value):
    """Yageo RC0805FR-07<value>L: 0805, 1 %, 0.125 W thick film ('4.7k' -> 4K7, '100k' -> 100K); the part number is from memory (verify before ordering)."""
    code = value.upper().replace('.', 'K')[:-1] if '.' in value else value.upper()
    return f'Yageo RC0805FR-07{code}L (0805, 1 %, 0.125 W thick film; verify before ordering)'
PARTS.update({r: SMD('Device:R', FP['R'], v, res_mpn(v), 0.6) for r, (v, a, b) in RES.items()})
# Bay B7 BOM: the items with no footprint of their own that plug into the board's parts ('on': those parts). Owned items keep the
# parts-and-connections.json status (the Pico E02, the GY-521 E03 and the P03 are the user's). Mounting hardware and printed parts are in
# pcb/fab/ORDERING.md (B9 puts them in the robot BOM).
BOM_EXTRAS = [
    dict(item='Raspberry Pi Pico (RP2040), owned (E02), with 2.54 mm male headers soldered on by the user', qty=1,
         mpn='Raspberry Pi SC0915 (bare Pico, no headers; owned, E02; solder two 1x20 male strips before it goes in the sockets)', on=['U1A', 'U1B']),
    dict(item='DRV8874 single brushed DC motor driver carrier', qty=2, mpn='Pololu 4035', on=['U3L', 'U3P', 'U4L', 'U4P']),
    dict(item='GY-521 MPU-6050 IMU module', qty=1, mpn='HiLetgo GY-521 (owned, E03)', on=['U2']),
    dict(item='P03 adjustable 5 V buck module, leads to the terminals', qty=1, mpn="P03 (owned; set 5.1 V on the bench first)", on=['U5A', 'U5B']),
    dict(item='ATO blade fuse 7.5 A (main)', qty=1, mpn='Littelfuse 0287007.PXCN (ATO 7.5 A; verify before ordering)', on=['F1']),
    dict(item='ATO blade fuse 5 A (motor branch)', qty=1, mpn='Littelfuse 0287005.PXCN (ATO 5 A; verify before ordering)', on=['F2']),
]

# Pico GPIO assignment (L5b): net -> Pico GP pin, the one place the map lives. Chosen for routing on the strip: the rear-edge connectors
# beside the GP0 (rear) row, the driver / right-end nets on the far (+v) end of both rows, the spare header on GP20-22 beside the front
# pocket. Hardware constraints (test_pico_gpio_constraints): SDA / SCL one I2C pair (I2C1 GP2 / GP3), the analog nets on GP26-28,
# each driver's IN1 / IN2 on the A / B outputs of one PWM slice (GP16 / GP17 slice 0, GP18 / GP19 slice 1). L5 map in the L6 report.
PICO_GPIO = {
    'XSHUT1': 'GP0', 'ENC_LA_3V3': 'GP1', 'SDA': 'GP2', 'SCL': 'GP3', 'ENC_LB_3V3': 'GP4', 'XSHUT3': 'GP5', 'BUMP_L': 'GP6',
    'DRV_SLEEP': 'GP7', 'DRV_FAULT': 'GP8', 'MOTOR_ON_N': 'GP9', 'XSHUT2': 'GP10', 'ENC_RA_3V3': 'GP11', 'ENC_RB_3V3': 'GP12',
    'XSHUT4': 'GP13', 'ARM': 'GP14', 'BUMP_R': 'GP15', 'L_IN1': 'GP16', 'L_IN2': 'GP17', 'R_IN1': 'GP18', 'R_IN2': 'GP19',
    'SPARE1': 'GP22', 'SPARE2': 'GP21', 'SPARE3': 'GP20', 'VSENSE': 'GP26', 'L_CS_ADC': 'GP27', 'R_CS_ADC': 'GP28',   # LIO-226: CS after its RC
}
def gp(net): return pico(PICO_GPIO[net])

# J1 pinout changed (PB20): pin 1 = GND, pin 2 = BAT+. Pin 1 is the south pin (v 73.0 since B12a's XT30; was 71.57), beside the free F.Cu
# west strip, K1.5 and the main B.Cu GND fill, so the battery return leaves on outer-layer copper; pin 2 (v 68.0) faces J2 / F1, next to
# BAT+'s fuse. B12a: the XT30UPB-M footprint's own '-' / '+' pads are pins 1 / 2, so the pinout holds. Until bay
# B6 it was the other way round (pin 1 BAT+), which boxed the return pin in between BAT+ and F1_OUT. The battery lead (B9 harness) and the
# sim / BRINGUP docs (B10) follow this table; the silk carries '+' / '-' at the pins (PIN_MARKS).
J1_PINOUT = {'1': 'GND', '2': 'BAT+'}
J1_GND_PIN, J1_BAT_PIN = ('J1', '1'), ('J1', '2')
def _nets():
    n = {}
    def add(net, *pins): n.setdefault(net, []).extend(pins)
    for r, (v, a, b) in RES.items(): add(a, (r, '1')); add(b, (r, '2'))
    add('BAT+', J1_BAT_PIN, ('F1', '1')); add('F1_OUT', ('F1', '2'), ('J2', '1'))
    add('VBAT_SW', ('J2', '2'), ('F2', '1'), ('U5A', '1'), ('TP1', '1'))
    add('F2_OUT', ('F2', '2'), ('K1', RELAY_PINS['COM']))
    add('VMOT', ('K1', RELAY_PINS['NO']), ('C1', '1'), ('C3', '1'), drv('L', 'VIN'), drv('R', 'VIN'), ('TP2', '1'))
    add('5V', ('U5B', '1'), ('J3', '1'), ('J4', '4'), ('J5', '4'), ('J13', '1'), ('J15', '5'), ('TP3', '1'), dio('D2', 'A'))
    add('VSYS', dio('D2', 'K'), pico('VSYS'))                                       # LIO-226: D2 (DNP) 5V -> VSYS
    add('3V3', pico('3V3_OUT'), imu('VCC'), *[(f'J{6 + i}', '1') for i in range(4)], ('J14', '1'), ('J15', '4'), ('TP4', '1'))
    if DRV_PMODE_NET == '3V3': add('3V3', drv('L', 'PMODE'), drv('R', 'PMODE'))
    gnd = [J1_GND_PIN, ('J3', '2'), ('J4', '3'), ('J5', '3'), ('J10', '2'), ('J11', '2'), ('J12', '2'), ('J14', '2'), ('J15', '6'),
           ('U5A', '2'), ('U5B', '2'), ('C1', '2'), ('C2', '2'), ('C3', '2'), ('C4', '2'), ('C5', '2'), ('K1', RELAY_PINS['COIL_N']), ('TP5', '1'),
           *[(f'J{6 + i}', '2') for i in range(4)], *[pico(g) for g in ('GND1', 'GND2', 'GND3', 'GND4', 'GND5', 'GND6', 'AGND', 'GND7')],
           imu('GND'), imu('AD0'), drv('L', 'GND'), drv('R', 'GND'), *[(f'H{i}', '1') for i in range(1, 5)], dio('D1', 'A'),
           *[(f'U{u}{row}', str(pin)) for u in (3, 4) for (row, pin) in DRV_EXTRA_GND],
           *[q(i, 'S') for i in range(1, 6)], *[(f'LED{i}', '1') for i in range(1, 4)]]
    if DRV_PMODE_NET == 'GND': gnd += [drv('L', 'PMODE'), drv('R', 'PMODE')]
    add('GND', *gnd)
    add('COIL_P', ('J13', '2'), ('K1', RELAY_PINS['COIL_P']), dio('D1', 'K'), q(5, 'G'))   # D1 cathode on coil+, anode on GND
    for side, j in (('L', 'J4'), ('R', 'J5')):
        add(f'{side}_M1', (j, '1'), drv(side, 'OUT1')); add(f'{side}_M2', (j, '2'), drv(side, 'OUT2'))
        add(f'{side}_CS', drv(side, 'CS')); add(f'{side}_IN1', drv(side, 'IN1')); add(f'{side}_IN2', drv(side, 'IN2'))
    for net in ('L_CS_ADC', 'R_CS_ADC', 'L_IN1', 'L_IN2', 'R_IN1', 'R_IN2'): add(net, gp(net))
    add('L_CS_ADC', ('C4', '1')); add('R_CS_ADC', ('C5', '1'))                      # LIO-226: the RC filters' caps (R24 / R25 in RES)
    add('DRV_SLEEP', gp('DRV_SLEEP'), drv('L', 'SLEEP'), drv('R', 'SLEEP')); add('DRV_FAULT', gp('DRV_FAULT'), drv('L', 'FAULT'), drv('R', 'FAULT'))
    for i, (sig, j, pin) in enumerate((('LA', 'J4', '5'), ('LB', 'J4', '6'), ('RA', 'J5', '5'), ('RB', 'J5', '6')), 1):
        add(f'ENC_{sig}_5V', (j, pin), q(i, 'G')); add(f'ENC_{sig}_3V3', q(i, 'D'), gp(f'ENC_{sig}_3V3'))
    add('MOTOR_ON_N', q(5, 'D'), gp('MOTOR_ON_N'))
    add('SDA', gp('SDA'), imu('SDA'), *[(f'J{6 + i}', '3') for i in range(4)], ('J14', '3'))
    add('SCL', gp('SCL'), imu('SCL'), *[(f'J{6 + i}', '4') for i in range(4)], ('J14', '4'))
    for i in range(1, 5): add(f'XSHUT{i}', (f'J{5 + i}', '5'), gp(f'XSHUT{i}'))
    add('BUMP_L', ('J10', '1'), gp('BUMP_L')); add('BUMP_R', ('J11', '1'), gp('BUMP_R')); add('ARM', ('J12', '1'), gp('ARM'))
    add('VSENSE', gp('VSENSE'), ('C2', '1'))
    for i in range(1, 4): add(f'SPARE{i}', ('J15', str(i)), gp(f'SPARE{i}'))
    for i in range(1, 4): add(f'LED{i}_A', (f'LED{i}', '2'))
    return n

NETS = _nets()
NC = ([pico(n) for n in ('RUN', 'ADC_VREF', '3V3_EN', 'VBUS')] + [imu(n) for n in ('XDA', 'XCL', 'INT')]
      + [('K1', RELAY_PINS['NC'])] + [(f'U{u}{row}', str(pin)) for u in (3, 4) for (row, pin) in DRV_UNUSED])

NETCLASS_POWER = ['BAT+', 'F1_OUT', 'VBAT_SW', 'F2_OUT', 'VMOT', '5V', 'GND', 'L_M1', 'L_M2', 'R_M1', 'R_M2']
CURRENT_A = {'BAT+': 7.5, 'F1_OUT': 7.5, 'VBAT_SW': 7.5, 'F2_OUT': 5.0, 'VMOT': 5.0, '5V': 4.71, 'GND': 7.5,
             'L_M1': 2.1, 'L_M2': 2.1, 'R_M1': 2.1, 'R_M2': 2.1}
# '5V' (LIO-227): the P03's whole output, the Jetson's 4.5 A + the 5V taps (TAP_LOAD_A: relay coil 83 mA, J15 spare 0.10 A, encoders
# 20 mA, LED2 3.5 mA = 0.207 A, rounded up to 0.21; test_board_spec checks the sum). LIO-226: not D2's PICO_VSYS_A - the DNP VSYS diode,
# fitted, moves the Pico's draw off the Jetson's USB port (inside the Jetson's 4.5 A) onto its tap.
# Net classes. Ruling R4, recomputed for the ordered copper (bay B4, copper.py: 1 oz outer, 0.5 oz inner, IPC-2221 at 10 degC rise):
# every power trunk (BAT+, F1_OUT, VBAT_SW, F2_OUT, VMOT, 5V; GND on the In1 plane) is a copper zone, POWER_ZONES below. The Power class
# keeps 3.0 mm for the short stubs B5 may add where every pad has >= 3.96 mm pitch: two 1-oz outer layers in parallel, 8.77 A > 7.5 A at
# 10 degC (copper.capacity_a({'F.Cu': 3.0, 'B.Cu': 3.0}), rated on the combined cross-section, LIO-227: summing the layers' 5.3 A each
# gave 10.6 A); one layer alone carries 5.3 A. PowerTap 0.8 mm (2.0 A on 1 oz outer, 0.62 A on 0.5 oz In2) feeds the small
# loads off the trunks; Motor 1.0 mm carries 2.1 A on 1 oz (IPC width 0.84 mm).
# Default (every other net, the signals; ruling PB17): 0.15 mm track, 0.15 mm clearance, so two tracks pass between 2.54 mm header pins
# (1.7 mm pads, 0.84 mm gap: 2 x 0.15 + 3 x 0.15 = 0.75). 0.15 mm carries about 0.6 A on 1 oz outer, 0.18 A on 0.5 oz inner (10 degC;
# LIO-232: was "0.25 A", the 20 degC figure), against signal currents of a few mA. B7: min track / space 6 / 6 mil, signal via 0.6 /
# 0.3 mm - JLC's standard multilayer capability (0.09 / 0.09).
DEFAULT_CLASS = dict(width=0.15, clearance=0.15)
NETCLASSES = {
    'Power':    dict(width=3.0, clearance=0.4, nets=['BAT+', 'F1_OUT', 'F2_OUT']),
    'PowerTap': dict(width=0.8, clearance=0.3, nets=['VBAT_SW', 'VMOT', '5V']),
    'Motor':    dict(width=1.0, clearance=1.0, nets=['L_M1', 'L_M2', 'R_M1', 'R_M2']),   # R16: 1 mm from every other net (PWM)
}
# KiCad custom DRC rules, written to r1_power_io.kicad_dru by make_schematic.py. The Motor class's 1.0 mm (R16) cannot hold between a
# motor pin and its neighbours in the same 2.54 mm header / carrier row (1.7 mm pads, 0.84 mm apart: J4, J5, U3P, U4P), so pad pairs
# within one footprint keep the class's former 0.3 mm; every track, via and zone keeps 1.0 mm from motor copper.
# Ruling PB21 (B6 fix 2): U3P.2 (DRV_L GND) can take one relief spoke per outer layer only - U3P.1 (VMOT) north, U3P.3 (L_M1, Motor
# 1.0 mm) south and the L_M1 track (u 49.85, held west of H3's washer) east leave the west side alone - so its GND patch joins it with one
# F.Cu spoke and the B.Cu GND fill with one B.Cu spoke, below KiCad's default of 2 (starved_thermal). The rule accepts 1 for that pad
# alone; ktest_power proves those spokes' current (pad metric) and ktest_pcb that no other pad is excused.
# LIO-226: JLCPCB's 4-layer minimums, so "DRC clean" means manufacturable rather than KiCad's defaults (annular ring 0.1, hole to hole
# 0.25, no silk rules): annular ring >= 0.13, plated hole >= 0.3, non-plated >= 0.5, hole edge to hole edge >= 0.5, silk lines and text
# strokes >= 0.12 (the stock footprints draw 0.12), silk >= 0.1 off the mask openings (not off the board edge: the stock silk of the
# edge headers sits 0.09-0.10 mm inside it, which the fab may clip: ORDERING.md). Copper to edge: JLCPCB needs 0.3, but a custom rule replaces KiCad's 0.5
# default (the zones would fill 0.2 mm nearer the edge), so the rule keeps the board's 0.5, which the router keeps too.
# LIO-232: kicad-cli reports a hole_to_hole, text_thickness or silk_clearance violation as a warning by default; those rules say (severity
# error), so a breach fails DRC like the others. LIO-236: the silk-to-mask rule's severity also makes a silk-to-silk silk_overlap an error;
# an assertion fails as an error anyway, so the silk-line rule's severity changes nothing.
DRC_RULES = '''(version 1)
(rule "motor pins within one footprint"
  (constraint clearance (min 0.3mm))
  (condition "A.Type == 'Pad' && B.Type == 'Pad' && A.Parent == B.Parent && (A.NetClass == 'Motor' || B.NetClass == 'Motor')"))
(rule "U3P.2 one spoke per outer layer"
  (constraint min_resolved_spokes 1)
  (condition "A.Type == 'Pad' && A.memberOfFootprint('U3P') && A.Pad_Number == '2'"))
(rule "JLCPCB annular ring"
  (constraint annular_width (min 0.13mm)))
(rule "JLCPCB plated hole"
  (condition "A.isPlated()")
  (constraint hole_size (min 0.3mm)))
(rule "JLCPCB non-plated hole"
  (condition "!A.isPlated()")
  (constraint hole_size (min 0.5mm)))
(rule "JLCPCB hole to hole"
  (severity error)
  (constraint hole_to_hole (min 0.5mm)))
(rule "JLCPCB copper to edge"
  (constraint edge_clearance (min 0.5mm)))
(rule "JLCPCB silk to mask"
  (severity error)
  (layer "?.Mask")
  (condition "!A.existsOnLayer('Edge.Cuts') && !B.existsOnLayer('Edge.Cuts')")
  (constraint silk_clearance (min 0.1mm)))
(rule "JLCPCB silk text stroke"
  (severity error)
  (condition "A.Type == 'Text' && (A.Layer == 'F.Silkscreen' || A.Layer == 'B.Silkscreen')")
  (constraint text_thickness (min 0.12mm)))
(rule "JLCPCB silk line width"
  (severity error)
  (condition "A.Type == 'Graphic' && (A.Layer == 'F.Silkscreen' || A.Layer == 'B.Silkscreen')")
  (constraint assertion "A.Line_Width >= 0.12mm"))
'''
POWER_TRUNK = {   # pads the high-current path must join with >= 3 mm zone copper on every trunk layer (zone_check; B6 sizes it by IPC-2221)
    'BAT+': [J1_BAT_PIN, ('F1', '1')],                  # bay B4: the battery-side nets are zone trunks too (PB20: J1.2)
    'F1_OUT': [('F1', '2'), ('J2', '1')],
    'F2_OUT': [('F2', '2'), ('K1', RELAY_PINS['COM'])],
    'VBAT_SW': [('J2', '2'), ('F2', '1'), ('U5A', '1')],
    'VMOT': [('K1', RELAY_PINS['NO']), ('C1', '1'), drv('L', 'VIN'), drv('R', 'VIN')],
    '5V': [('U5B', '1'), ('J3', '1')],
    'GND': [J1_GND_PIN, ('J3', '2'), ('U5A', '2'), ('U5B', '2'), ('C1', '2'), drv('L', 'GND'), drv('R', 'GND')],
}
# Each trunk pin's own load (ruling PB12, B4 fix 1): what its thermal spokes must carry, over its layers (zone_check.pin_spoke_capacity).
# A fuse contact (4 pads) is one pin. Any trunk pin not listed carries CURRENT_A[net] (pad_current).
# P03_OUT_V is the design nominal; the bench trim is 5.1 V (BRINGUP.md section 5, BOM_EXTRAS), raised up to V5_MAX if the Jetson's barrel
# plug reads under 5.0 V under load (BRINGUP.md section 7 step 3; about 65 mOhm, 0.3 V at 4.5 A, from the P03 to the plug).
# P03_VIN_MIN is the firmware's low-battery stop. BRINGUP.md section 5 tests the P03 at 6.0 V in, 4.71 A out (P03_OUT_A; LIO-234) with
# its input current read against P03_EFF (the XL4015 needs about 0.6-0.8 V of headroom; motor starts sag the pack 0.3-0.5 V more). If it fails, the stop (BRINGUP.md section 8, BRINGUP-NOTES.md) rises
# to the lowest passing input + 0.5 V and P03_VIN_MIN is set to match (LIO-229).
P03_OUT_V, P03_OUT_A = 5.0, CURRENT_A['5V']     # the P03 regulator's output: the Jetson's 4.5 A + the 5V taps, 4.71 A (LIO-227)
P03_VIN_MIN, P03_EFF = 6.0, 0.85     # its lowest input (the 6-cell NiMH pack near its 1.0 V/cell cut-off; full charge VBAT_MAX 8.7 V) and its
                                     # efficiency there: an XL4015 buck near dropout at 4.7 A, 0.85 assumed (LIO-227; was 0.9) - verify on the bench
P03_IN_A = P03_OUT_V * P03_OUT_A / (P03_VIN_MIN * P03_EFF)      # 4.62 A drawn through U5A
# (at the 5.1 V bench trim U5A carries about 4.71 A, at V5_MAX 4.85 A: its margins stay at +28 % or more)
PAD_CURRENT_A = {
    ('J1', '1'): 7.5, ('J1', '2'): 7.5,       # battery connector: the whole pack current, F1's 7.5 A rating
    ('J2', '1'): 7.5, ('J2', '2'): 7.5,       # master rocker: the same current through the switch
    ('F1', '1'): 7.5, ('F1', '2'): 7.5,       # F1 contacts: 7.5 A fuse
    ('F2', '1'): 5.0, ('F2', '2'): 5.0,       # F2 contacts: the motors' 5 A fuse
    ('K1', RELAY_PINS['COM']): 5.0, ('K1', RELAY_PINS['NO']): 5.0,   # relay contacts switch the motor supply (behind F2)
    ('J3', '1'): 4.5, ('J3', '2'): 4.5,       # Jetson 5 V lead: the Jetson's 4.5 A alone
    ('U5A', '1'): P03_IN_A, ('U5A', '2'): P03_IN_A,       # P03 input: 5 V x 4.71 A / (6.0 V x 0.85)
    ('U5B', '1'): P03_OUT_A, ('U5B', '2'): P03_OUT_A,     # P03 output: the Jetson + the 5V taps
    ('U3P', '1'): 2.1, ('U3P', '2'): 2.1, ('U4P', '1'): 2.1, ('U4P', '2'): 2.1,   # each driver's VIN and GND: its 2.1 A motor channel
    ('C1', '1'): 2.1, ('C1', '2'): 2.1,       # bulk capacitor: up to one driver's current in ripple
}
def pad_current(ref, pin):
    """A a trunk pin's spokes must carry: PAD_CURRENT_A, else its net's CURRENT_A."""
    if (ref, pin) in PAD_CURRENT_A: return PAD_CURRENT_A[(ref, pin)]
    return CURRENT_A[next(n for n, pins in NETS.items() if (ref, pin) in pins)]
# Bay B6: trunk pins whose PB12 figure (zone_check.pin_spoke_capacity, via barrels counted) is below pad_current pending a controller
# ruling - the only exceptions route.quality and the spoke tests allow, each still required to be short (so a fixed pin leaves the list).
# PB20 fixed the battery corner (J1.2's entry is gone). B6 fix 1 found U3P.2 short (its F.Cu patch stopped 0.05 mm off the pad, so no
# spoke); ruling PB21 (B6 fix 2) drew the patch to the pad with 90 degree spokes and a second stitching via (PAD_SPOKE_ANGLE, STITCH).
PAD_PENDING = {}
# Stitching vias excused from ktest_pcb's >= 2 live layers rule pending a ruling, each with its pending pin (still required to fail the
# rule, so a fixed via leaves the list). B6 fix 2: empty (U3P.2's patch vias are live).
STITCH_PENDING = {}
# Relief spoke angle (degrees) per pad, where KiCad's default (45 for a round pad) leaves no spoke into a pin's zone: build_pcb sets it.
# PB21: U3P.2's only free side is the west, where its GND patch lies; at 45 degrees its spokes run into U3P.1's / U3P.3's clearances.
PAD_SPOKE_ANGLE = {('U3P', '2'): 90.0}
# The pin each GND_PATCHES patch serves, in GND_PATCHES order (ktest_pcb: every patch joins its pin through real spokes on the filled
# board, and its stitching vias tie live copper).
GND_PATCH_PINS = [J1_GND_PIN, drv('L', 'GND')]
# Bay B6 (carry item 4): the small loads the supplies feed through tracks - each tap pad's own current (A), so ktest_power can sum the
# load behind every tap track and via (power_check.track_loads). TAP_NETS: the supplies with such taps; TAP_SOURCE: where a tap net
# with no trunk zone is fed. Sources marked "verify" are manufacturer figures not in the repo (quoted from memory, check before B9).
TAP_NETS = ('VBAT_SW', 'VMOT', '5V', '3V3', 'VSYS')
TAP_SOURCE = {'3V3': pico('3V3_OUT'),       # the Pico's 3.3 V regulator: 300 mA for external loads (Pico datasheet, verify)
              'VSYS': dio('D2', 'K')}       # LIO-232: VSYS is fed through the DNP D2 (if fitted); its load is the Pico's VSYS pin
VBAT_MAX, V5_MAX, LED_VF_MIN = 8.7, 5.25, 1.8   # spec 2026-10-04 s.6: VBAT_SW 6.0-8.7 V; the P03 5 V set +5 %; 0805 LED V_F, low end
ENC_SUPPLY_A = 0.010    # Pololu 4867 (25D, 48 CPR encoder): its Hall sensors draw at most 10 mA (Pololu product page, verify)
RELAY_COIL_A = V5_MAX / 63.0   # Omron G5LE-1 DC5 coil: 63 ohm, 79.4 mA rated at 5 V (Omron datasheet, verify) -> 83 mA at 5.25 V
IMU_A = 0.010           # GY-521: MPU-6050 3.9 mA (InvenSense datasheet) + the module's LDO and power LED, rounded up
TOF_A = 0.040           # Pololu 2490 VL53L0X carrier: ranging peaks near 40 mA (ST datasheet: 19 mA average, verify); spec 2026-10-04
                        # s.6 budgets IMU + 4 ToF at about 100 mA average
SPARE_5V_A, SPARE_3V3_A = 0.10, 0.02   # budgets of the DNP spare rows' supply pins (J15.5 5V, J14.1 / J15.4 3V3): a fitted load above
                                       # them needs this check re-run
PICO_VSYS_A = 0.20      # LIO-226, D2 fitted (DNP as delivered): the Pico (~25 mA) and its 3V3 loads here (~0.21 A, TAP_LOAD_A) at 3.3 V
                        # through its buck-boost (~90 %) from ~4.5 V (5 V less the SS14's drop). LIO-232: a 0.15 mm In2 track carries 0.18 A
                        # at 10 degC (not the 0.25 A once quoted, the 20 degC figure), so VSYS's scripted route is 0.25 mm on In2 (0.27 A)
def _lamp_a(v, ohm): return (v - LED_VF_MIN) / ohm
TAP_LOAD_A = {
    **{(f'TP{i}', '1'): 0.0 for i in range(1, 5)},                 # probe pads: no load (TP5 is GND)
    ('R11', '1'): VBAT_MAX / (100e3 + 33e3),                       # VSENSE divider top (R11 + R12): 65 uA
    ('R21', '1'): _lamp_a(VBAT_MAX, 3.3e3),                        # LED1 lamp on VBAT_SW: 2.1 mA
    ('R23', '1'): _lamp_a(VBAT_MAX, 3.3e3),                        # LED3 lamp on VMOT: 2.1 mA
    ('C3', '1'): 0.0,                # DRV_L's bulk cap, zone-fed: no DC load (its ripple share: power_check.c3_ripple)
    ('R22', '1'): _lamp_a(V5_MAX, 1e3),                            # LED2 lamp on 5V: 3.5 mA
    ('J4', '4'): ENC_SUPPLY_A, ('J5', '4'): ENC_SUPPLY_A,           # motor encoder supplies
    ('J13', '1'): RELAY_COIL_A,      # the motor toggle loop feeds K1's coil (J13.2 -> COIL_P)
    ('J15', '5'): SPARE_5V_A, ('J14', '1'): SPARE_3V3_A, ('J15', '4'): SPARE_3V3_A,
    dio('D2', 'A'): PICO_VSYS_A,                                   # LIO-226: the DNP VSYS diode, if fitted
    pico('VSYS'): PICO_VSYS_A,                                     # LIO-232: ... and the Pico's VSYS pin behind it (VSYS's tap tracks)
    imu('VCC'): IMU_A, **{(f'J{6 + i}', '1'): TOF_A for i in range(4)},
    drv('L', 'PMODE'): 0.0001, drv('R', 'PMODE'): 0.0001,          # DRV8874 PMODE: a logic input (uA)
    **{(r, '2'): 3.3 / 10e3 for r in ('R2', 'R4', 'R6', 'R8', 'R10', 'R18', 'R19', 'R20', 'R26')},   # 10 k pull-ups to 3V3 (I2C, ARM: LIO-226)
}
# Bay B6 (carry item 6): C3's ripple share. C3 = Panasonic EEE-FK1C101P (FK, 100 uF 16 V, 6.3 x 5.8 mm; PB20 - was EEE-FK1E101P,
# 6.3 x 7.7 mm, 0.34 ohm / 0.23 A), C1 = EEU-FR1E471 (FR, 470 uF 25 V). Datasheet figures at 100 kHz / 20 degC (ESR) and 105 degC
# (ripple), quoted from memory: verify before ordering. The ripple rating is the 100 kHz figure, not derated to DRV_PWM_HZ (an
# electrolytic's frequency multiplier is below 1 under 100 kHz). LIO-227: C3's share is 0.10 A with slow-decay PWM and about 0.20 A with
# fast decay, over the rating, so the firmware must PWM with slow decay (power_check.c3_ripple, BRINGUP-NOTES.md).
C3_ESR_OHM, C3_RIPPLE_RATING_A = 0.44, 0.16
C1_ESR_OHM = 0.04
DRV_PWM_HZ = 20e3          # the motor PWM the firmware is expected to use (not in the repo; the DRV8874 takes up to 100 kHz)
C3_C1_PATH = dict(r_ohm=0.006, l_h=40e-9)   # the VMOT copper between C3 (U3P.1) and C1: ~45 mm of the F.Cu pour over the In1 plane
# Datasheet height tolerances of parts whose nominal height (PARTS h) is near a limit (carry item 6, for B9). PB20 (d): the 6.3 x 7.7 mm
# can reached MODULE_CLEARANCE (8.0 mm, the free height under a plugged module) with no margin; the 6.3 x 5.8 mm can is 5.8 +/- 0.3 mm,
# under C3_H_MAX (worst case <= 7.5 mm, 0.5 mm below the limit).
PART_H_TOL = {'C3': 0.3}
C3_H_MAX = 7.5
# Robot-CAD (cad/component-manifest.json) ids the real board replaces in B8: the B1 placeholders of the bay board. Every id exists in the current
# CAD (test_removed_cad_ids_exist). extract_outline.py skips these when it builds the keep-outs (the placeholder board, its modules, its four M3 spacers
# and screws and the two clamp spacers are what the outline replaces); everything else that stays counts: the motors (notches), the battery tray
# (rear edge) and the P03 regulator (front edge) are the ones that bite.
# NOT listed on purpose (extract_outline.py excludes them from the keep-outs by name, with the reasons): the heat-set inserts insert_bay_0..3, the
# printed bearing ribs bay_rib_0/1 and the Jetson standoff / screw stacks jetson_standoff_1/3, jetson_screw_1/3 at the clamp. The harness wires W*
# are regenerated from the new connector list.
REMOVED_CAD_IDS = [
    'bay_board', 'bay_pico', 'bay_drv_0', 'bay_drv_1', 'bay_imu', 'pico_usb_plug',
    *[f'bay_spacer_{i}' for i in range(4)], *[f'bay_screw_{i}' for i in range(4)],
    'bay_clamp_spacer_1', 'bay_clamp_spacer_3',
]
@lru_cache(maxsize=None)
def _symbols(lib):
    return {c[1]: c for c in find(parse((tools.SYMBOL_DIR / f'{lib}.kicad_sym').read_text(encoding='utf-8'))[0], 'symbol')}
def symbol_pin_numbers(lib_id):
    lib, name = lib_id.split(':')
    syms = _symbols(lib)
    s = syms[name]
    if find(s, 'extends'): s = syms[find(s, 'extends')[0][1]]
    return {str(find(p, 'number')[0][1]) for sub in find(s, 'symbol') for p in find(sub, 'pin')}
# ---------------------------------------------------------------------------------------------------------------------------------
# Task L5: placement on the strip.
def rot(r, x, y):
    c, s = round(math.cos(math.radians(r)), 12), round(math.sin(math.radians(r)), 12)
    return (x * c + y * s, -x * s + y * c)
def local_to_board(place, xy):
    u, v, r = place
    dx, dy = rot(r, *xy)
    return (u + dx, v + dy)
def rect_pts(rect):
    u0, v0, u1, v1 = rect
    return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]

# Plug-in modules (ruling R8). Frame = the FIRST socket's own footprint frame at rotation 0 (pin 1 at (0, 0), pins along +y), seen from
# above with the module component side up. 'rows': socket -> its pin-1 position in that frame; 'body': module outline (x0, y0, x1, y1).
# Every socket of a module has the same rotation and sits at local_to_board(first socket, row offset), so the real module drops in.
# The body is drawn on F.Fab, not as a courtyard (L5): the reflowed SMD parts may sit under a module (MODULE_CLEARANCE below).
PICO_ROW_SPACING = 17.78           # Raspberry Pi Pico datasheet: rows 0.7 in apart, holes 1.61 mm from the long edges, board 51 x 21 mm
MODULES = {
    # Pico, top view with the micro-USB end up: pin 1 (GP0) top of the LEFT row, pin 40 (VBUS) top of the right row (U1B socket pin 1).
    # Body 21 x 51 mm; the first/last holes are 1.37 mm from the short edges; +1.3 mm at the USB end for the receptacle's overhang.
    'Pico': dict(rows={'U1A': (0.0, 0.0), 'U1B': (PICO_ROW_SPACING, 0.0)}, body=(-1.61, -2.67, PICO_ROW_SPACING + 1.61, 19 * 2.54 + 1.37)),
    # Pololu 4035, pcb/reference/pololu-4035.txt: VM row (L) left, VIN row (P) right, pin 1 of both at the same end; 15.24 x 17.78 mm,
    # holes 1.27 mm from every edge.
    **{f'DRV_{s}': dict(rows={f'U{n}L': (0.0, 0.0), f'U{n}P': (DRV_ROW_SPACING, 0.0)}, body=(-1.27, -1.27, DRV_ROW_SPACING + 1.27, 6 * 2.54 + 1.27))
       for s, n in (('L', 3), ('R', 4))},
    # GY-521 (MPU-6050), top view with the header along the bottom edge reads VCC ... INT left to right (the order of the silk labels), so
    # in the socket frame the 21.2 x 16.4 mm board lies on the +x side of the row. Row 1.27 mm from the edge, centred along the 21.2 mm side.
    'IMU': dict(rows={'U2': (0.0, 0.0)}, body=(-1.27, -(21.2 - 7 * 2.54) / 2, 16.4 - 1.27, 7 * 2.54 + (21.2 - 7 * 2.54) / 2)),
}
USB_PLUG_W = 12.0                  # mm: micro-B plug over-mould (about 10.6 mm) plus margin; the Pico's USB end must see a clear path this wide

MODULE_CLEARANCE = 8.0             # mm: free height under a plugged module (socket body 8.5 mm); only top-side SMD parts lower than this go
                                   # there (the tallest: C3's 5.8 mm V-chip, PB16 / PB20)

# Spec section 3: an SMD pad keeps >= SMD_TALL_GAP (2.0 mm) from the pads of the tall through-hole parts (relay, fuse holders, C1,
# sockets, VH connectors) so the reflow / rework hot air is not blocked. L5 fix 1 (ruling LP9) applies it to EVERY through-hole pad -
# XH and pin headers, screw terminals included - except the M3 mounting holes' own pads (GND rings, nothing stands on them).
THT_GAP_EXEMPT = ['H1', 'H2', 'H3', 'H4']
SMD_GAP, SMD_TALL_GAP = 1.0, 2.0   # mm: SMD courtyard to SMD courtyard; SMD pad to any through-hole pad but THT_GAP_EXEMPT

# Silkscreen (spec 2026-10-06 section 3, L5 fix 1): every part but the M3 holes carries one F.SilkS label "<ref> <text>", 0.8 mm high with a
# 0.15 mm stroke, placed by build_pcb clear of pads, of other silk and of every part body: <text> is the value for the SMD parts, the
# relay, C1 and the fuse holders, and a short function word for connectors, terminals and module sockets. The stock footprints carry the
# polarity marks (LED / SS14 cathode bar, SOT-23 pin-1 mark, C1 "+"); ktest_pcb asserts both.
SILK_TEXT_H, SILK_TEXT_W, SILK_STROKE = 0.8, 0.7, 0.15
SILK_FUNCTION = {'J1': 'BATT', 'J2': 'ROCKER', 'J3': 'JETSON 5V', 'J4': 'MOTOR L', 'J5': 'MOTOR R', 'J6': 'CLIFF1', 'J7': 'CLIFF2',
                 'J8': 'CLIFF3', 'J9': 'CLIFF4', 'J10': 'BMP L', 'J11': 'BMP R', 'J12': 'ARM', 'J13': 'LOOP', 'J14': 'I2C',
                 'J15': 'SPARE GPIO (DNP)', 'U5A': 'P03 IN', 'U5B': 'P03 OUT', 'U1A': 'PICO', 'U1B': 'PICO', 'U2': 'GY-521', 'U3L': 'DRV L',
                 'U3P': 'DRV L', 'U4L': 'DRV R', 'U4P': 'DRV R'}
def silk_label(ref):
    """The F.SilkS label of a part (None for the M3 holes)."""
    if ref.startswith('H') and ref[1:].isdigit(): return None
    return f"{ref} {SILK_FUNCTION.get(ref, PARTS[ref]['value'])}"
POLARISED = ['LED1', 'LED2', 'LED3', 'D1', 'D2', 'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'C1', 'C3']
# PB20: connector pin marks build_pcb draws on F.SilkS: '+' at a supply pin, '-' at a GND pin. Each mark is PIN_MARK_LEN strokes (a '+'
# two crossing, a '-' one along the pin row, the way the J1 label reads) at the pin centre + PIN_MARK_AT[ref][pin] (board frame mm),
# outside the part's F.Fab housing, so it stays visible once the part is soldered and mated, nearer its own pin than any other pad on the
# board, off every other part's body (its F.Fab drawings, its pads, J4 / J5's C18 socket) - ktest_pcb test_polarity_marks_on_silk.
#  - J1 (PB20, J1's pinout changed: J1_PINOUT): '+' at BAT+ (pin 2), '-' at GND (pin 1). B12a: the XT30 housing covers both pin ends
#    (J2 north, K1 south), so the marks sit beside the pins on the +u side, 0.4 mm off J1's silk outline; the footprint's own '+' / '-'
#    texts (past the housing ends, under J2 and K1) go to F.Fab (build_pcb).
#  - B12a (final review N2): '+' at pin 1 of both P03 terminals (U5A.1 VBAT_SW, U5B.1 5V: the screw nearer the robot centre line) and of
#    the Jetson 5 V header (J3.1). U5A.1's behind the terminal (its -u side) in the 1 mm slot between U5A's body and the end of J4's
#    pin row, south of J4's header body: inside J4's courtyard margin, off its body, pins and the C18 socket (pin row +- 7.62 mm), 0.6 mm
#    nearer U5A.1 than J4.6 (a 0.7 mm J4 move for room put J4 under the IMU module and its plug corridor into U2 / R3). U5B.1's past the
#    terminal's +v end (pin 1's end: TP3 and H3's vias leave its -u side no spot nearer U5B.1 than another pad), 2 mm toward the front
#    edge so U5B's label fits beside it (SILK_LABEL_AT). J3.1's north of the Micro-Fit housing.
#  - LIO-242: a 'VIN' text (any mark but '+' / '-' is drawn as a SILK_TEXT_H text) at the DRV8874 carriers' VIN pin (U3P.1, U4P.1). The
#    Pololu 4035 is symmetric: turned 180 degrees it fits the same two sockets (carrier row L pin k in socket row P pin 8 - k), which puts
#    its VREF on VMOT and its GNDs on DRV_SLEEP / DRV_FAULT. The mark sits 2.1 mm past the carrier's pin-1 end (-v), off its body.
PIN_MARKS = {'J1': {J1_BAT_PIN[1]: '+', J1_GND_PIN[1]: '-'}, 'U5A': {'1': '+'}, 'U5B': {'1': '+'}, 'J3': {'1': '+'},
             'U3P': {str(DRV_PINS['VIN'][1]): 'VIN'}, 'U4P': {str(DRV_PINS['VIN'][1]): 'VIN'}}
PIN_MARK_AT = {'J1': {J1_BAT_PIN[1]: (3.5, 0.0), J1_GND_PIN[1]: (3.5, 0.0)}, 'U5A': {'1': (-5.85, -0.5)}, 'U5B': {'1': (2.0, 3.3)},
               'J3': {'1': (0.0, -1.7)}, 'U3P': {'1': (0.0, -2.1)}, 'U4P': {'1': (0.0, -2.1)}}
PIN_MARK_LEN = 0.7      # B12a: 0.8 -> 0.7 mm, so U5A.1's '+' (0.85 mm over the stroke ends) fits its 0.95 mm slot with 0.05 mm a side
# Labels placed by hand (B12a): ref -> (u, v, angle) of the label's glyph-box centre, held to place_silk_labels' rules (build_pcb asserts
# them) and placed before the search. U5B's only free side is the strip past its +v end (u 62.6 - 73.5, between its body and J8 / J15),
# which its '+' mark now shares: the search's offsets put the 7.1 mm label over the mark, so it sits at the strip's -u end by hand
# (0.22 mm from TP3's label, 0.27 mm from the mark).
# LIO-232: the search stacked C4's label (south of C4) 0.28 mm over R24's (east of R24, upright), their text cells overlapping. C4's
# moves 0.1 mm north - its own pads' SILK_PAD_GAP allows no more - and R24's 0.4 mm south: 0.78 mm between the glyph boxes.
SILK_LABEL_AT = {'U5B': (66.205, 44.355, 0), 'C4': (53.0, 48.769, 0), 'R24': (53.089, 52.365, 90)}

# ---------------------------------------------------------------------------------------------------------------------------------
# Bay B3: connectors on the bay board's edges. EDGE_CONNECTORS: ref -> entry direction, the way its cable leaves the board ('+u' front,
# '-u' rear, '-v' left, '+v' right). The P03 terminals are edge connectors too (TERMINALS, their wire entry faces the P03). The DNP spare
# rows J14 / J15 (ruling PB8) have no edge.
EDGE_CONNECTORS = {
    'J1': '-u', 'J2': '-u', 'J13': '-u', 'J12': '-u', 'J6': '-u', 'J7': '-u',     # behind the carrier, rear edge (battery side), latches outward
    'J3': '+v',                    # Jetson 5 V (PB5 fallback): in the band, out of the right edge toward the W04 corridor (B12a: Micro-Fit
                                   # right-angle, its mating face at the edge)
    'J4': '-v', 'J5': '+v',        # motor pigtails (C18, PB25), toward their motor notches (the socket lies on the board, MATED_REACH)
    'J8': '+u', 'J9': '+u',        # front cliffs, tongue front edge; the leads rise over the P03 under the carrier
    'J10': '-v', 'J11': '+v',      # bumpers, side edges under the carrier
    'U5A': '+u', 'U5B': '+u',      # P03 terminals, wire entry facing the P03
}
TERMINALS = {r: EDGE_CONNECTORS[r] for r in ('U5A', 'U5B')}
# Entry side of each connector footprint in its own frame (unit vector): vertical JST VH / XH -> the latch side (local -y: the lock can be
# pressed from the edge and the wires leave without crossing the header); side-entry VH and the MKDS terminals -> the plug / wire face
# (local +y, checked on the KiCad 3D models); a right-angle pin header -> the pin tips (local +x). B12a: the vertical XT30 has no latch -
# its local -y long side faces the edge (the lead leaves its top toward it, as the VH's); the Micro-Fit right-angle header's mating face
# is local -y (its F.Fab body runs from the pins at y 0 to the face at y -8.92, the peg between).
ENTRY_LOCAL = {
    'JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical': (0, -1), 'JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical': (0, -1),
    'JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical': (0, -1), 'JST_VH_B2P-VH_1x02_P3.96mm_Vertical': (0, -1),
    'JST_VH_S2P-VH_1x02_P3.96mm_Horizontal': (0, 1), 'PinHeader_1x06_P2.54mm_Horizontal': (1, 0),
    'TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal': (0, 1),
    'AMASS_XT30UPB-M_1x02_P5.0mm_Vertical': (0, -1), 'Molex_Micro-Fit_3.0_43650-0200_1x02_P3.00mm_Horizontal': (0, -1),
}
# Wire-bend allowance above a vertical mated plug (the wires leave its top and turn 90 deg toward the edge).
CABLE_BEND_MM = {'XH': 5.0, 'PinHeader': 5.0, 'VH': 7.0, 'XT30': 7.0, 'MicroFit': 7.0}   # B12a: 18 AWG silicone, as the VH leads
# Mated height above the board top (header + plug), mm, from the datasheets:
#  - JST XH top entry: "mounting height 9.8 mm" (JST eXH catalog);
#  - JST VH top entry 16.5 mm, side entry 10.5 mm (JST VH catalog: "the above-board heights of the mated VH connectors are 16.5 mm for
#    the top entry and 10.5 mm for the side entry type");
#  - 0.1 in pin header with the C18 pigtail's right-angle 1x6 socket (PB25; sized as the 0.1 in DuPont-style housing 14.0 x 15.24 x
#    2.6 mm; the motors' factory cables mate at the inline joints off the board): right-angle, the socket lies on the board centred on
#    the pins (KiCad STEP: pins at z 1.27) -> 2.6 mm. (Vertical it would stand 2.54 + 14.0 = 16.54 mm, over UNDER_CARRIER_MAX_H before
#    the wire bend: hence the right-angle part.)
#  - Phoenix MKDS 1,5/2-5,08 terminals: the body (PARTS h, 12 mm); the wires enter horizontally below its top.
#  - B12a, AMASS XT30UPB-M + XT30U-F (J1): the PCB male about 11 mm over the board, the female's 15-16 mm housing over it with about 5 mm
#    engaged, its solder cups and heat-shrink on top: 22 mm (from memory: verify on the AMASS drawings).
#  - B12a, Molex 43650-0200 + 43645-0200 (J3): the right-angle header's housing about 10 mm over the board; the receptacle enters its face
#    horizontally below that top: 10 mm (verify on the Molex drawings). Its reach past the face is OFFBOARD_REACH.
MATED_H = {**{r: 9.8 for r in ('J6', 'J7', 'J8', 'J9', 'J10', 'J11', 'J12', 'J13')}, 'J1': 22.0, 'J2': 16.5, 'J3': 10.0,
           'J4': 2.6, 'J5': 2.6, 'U5A': 12.0, 'U5B': 12.0}
# B12a: a right-angle header whose mated plug sticks out past the board edge (not lying on the board, so not MATED_REACH): how far the plug
# and its crimp ends reach past the header's mating face along the entry direction. J3: the 43645-0200 receptacle stands about 6 mm proud
# of the 43650 header's face once latched, plus the wire's crimp barrels and strain relief: 10 mm (verify). The lead then bends upward
# (OFFBOARD_BEND_MM for 18 AWG) toward the W04 corridor.
OFFBOARD_REACH = {'J3': 10.0}
OFFBOARD_BEND_MM = 8.0
# Ruling PB6: how far the mated plug of a right-angle header reaches past its courtyard in the entry direction (it lies on the board there):
# the 14.0 mm housing seated on the header body face (local x 4.04, KiCad footprint / STEP) ends at x 18.04; the courtyard ends at
# x 10.59 -> 7.45 mm.
MATED_REACH = {'J4': 7.45, 'J5': 7.45}
# B3 fix 1: the motors' factory cables. Each leaves its motor at a fixed point - on the motor axis (robot x 0 -> u 66.5: the centre of
# motor_left / motor_right in cad/component-manifest.json, Pololu 4867 envelope x -12.5..12.5), on the inner end face (|y| 35 -> v 15 / 85,
# the box face nearest the board) and 6 mm over the board top (z 43, where cad/build_robot.py starts the motor_cable_* route; the axis is
# z 40) - and runs beside that face to the far end of its lay-flat plug. (u, v, height over the board top), keyed by the connector.
MOTOR_CABLE_EXIT = {'J4': (66.5, 15.0, round(43.0 - Z_PCB_TOP, 3)), 'J5': (66.5, 85.0, round(43.0 - Z_PCB_TOP, 3))}
# Cable section as a swept box: the six loose colour-coded 20 cm leads (Pololu 37D encoder motor page; the gauge is not published) laid flat
# side by side, about 6 mm wide and 2 mm thick (the CAD draws the bundle as a 4 mm round wire).
MOTOR_CABLE_W, MOTOR_CABLE_T = 6.0, 2.0
def connector_family(ref):
    """'XH' | 'VH' | 'XT30' | 'MicroFit' | 'PinHeader' | 'Terminal', from the PARTS footprint name (B12a: J1 XT30, J2 VH, J3 MicroFit - three
    families whose plugs do not fit each other's headers)."""
    name = PARTS[ref]['fp'].split(':')[1]
    for key, fam in (('JST_XH', 'XH'), ('JST_VH', 'VH'), ('AMASS_XT30', 'XT30'), ('Molex_Micro-Fit_3.0', 'MicroFit'), ('PinHeader', 'PinHeader'),
                     ('TerminalBlock', 'Terminal')):
        if name.startswith(key): return fam
    raise KeyError(ref)

@lru_cache(maxsize=None)
def _footprint_geometry(fp_id):
    """(courtyard bbox (x0, y0, x1, y1) in the footprint's own frame, {pad number: (x, y) of its first pad}) from the .kicad_mod file
    (pure Python, so UNDER_CARRIER and pad_uv work in the CAD venv too)."""
    lib, name = fp_id.split(':')
    base = CUSTOM_FP_DIR if lib == 'R1' else tools.FOOTPRINT_DIR / f'{lib}.pretty'
    root = parse((base / f'{name}.kicad_mod').read_text(encoding='utf-8'))[0]
    xs, ys = [], []
    for kind in ('fp_line', 'fp_rect', 'fp_poly', 'fp_circle', 'fp_arc'):
        for item in find(root, kind):
            if not find(item, 'layer') or find(item, 'layer')[0][1] != 'F.CrtYd': continue
            if kind == 'fp_circle':
                (c,), (e,) = find(item, 'center'), find(item, 'end')
                r = math.hypot(float(e[1]) - float(c[1]), float(e[2]) - float(c[2]))
                xs += [float(c[1]) - r, float(c[1]) + r]; ys += [float(c[2]) - r, float(c[2]) + r]
                continue
            pts = [p for key in ('start', 'end', 'mid') for p in find(item, key)]
            for pl in find(item, 'pts'): pts += find(pl, 'xy')
            xs += [float(p[1]) for p in pts]; ys += [float(p[2]) for p in pts]
            if kind == 'fp_arc':   # the bulge: the arc's points at 0 / 90 / 180 / 270 deg about its centre
                for ex, ey in arc_extremes(*((float(find(item, k)[0][1]), float(find(item, k)[0][2])) for k in ('start', 'mid', 'end'))):
                    xs.append(ex); ys.append(ey)
    pads = {}
    for pad in find(root, 'pad'):
        at = find(pad, 'at')[0]
        pads.setdefault(str(pad[1]), (float(at[1]), float(at[2])))
    return (min(xs), min(ys), max(xs), max(ys)), pads
def arc_extremes(s, m, e):
    """The axis-extreme points (centre +- r along x or y) that lie on the arc from s through m to e."""
    (ax, ay), (bx, by), (cx, cy) = s, m, e
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12: return []   # collinear: a straight segment, no bulge
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r, ang = math.hypot(ax - ux, ay - uy), lambda p: math.atan2(p[1] - uy, p[0] - ux)
    a0, sweep = ang(s), (ang(e) - ang(s)) % math.tau
    if (ang(m) - a0) % math.tau > sweep: a0, sweep = ang(e), math.tau - sweep   # m is on the other way round: e -> s counter-clockwise
    return [(ux + r * math.cos(t), uy + r * math.sin(t)) for t in (0, math.pi / 2, math.pi, 3 * math.pi / 2) if (t - a0) % math.tau <= sweep]
def courtyard_box(ref):
    """(u0, v0, u1, v1): the board-frame bounding box of the part's courtyard as placed by PLACEMENT."""
    x0, y0, x1, y1 = _footprint_geometry(PARTS[ref]['fp'])[0]
    pts = [local_to_board(PLACEMENT[ref], p) for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))
def pad_uv(ref, pin):
    """Board (u, v) of the part's pad `pin` (the first pad with that number) as placed by PLACEMENT."""
    return local_to_board(PLACEMENT[ref], _footprint_geometry(PARTS[ref]['fp'])[1][str(pin)])
def UNDER_CARRIER(ref):
    """True when the part's courtyard reaches u >= CARRIER_U (under the Jetson carrier, where the 16.27 mm limit applies)."""
    return courtyard_box(ref)[2] >= CARRIER_U

# Placement (u, v, rotation), bay B3 (spec 2026-10-08-r1-bay-board; rulings PB4-PB9). Board u 0.1..74 (rear to front), v 0..100 (left
# to right); the carrier zone starts at u 28; the tongue (u > 52.7) lies between the motor notches (v 16.6..83.4).
#  - band behind the carrier (u < 28, full height, exposed with the lid off): the rear-edge XH row J13 / J6 / J7 / J12 beside the Pico
#    (U1A u 8.8: its right socket's courtyard reaches u 28.38, ruling PB7), micro-USB out of the left edge; past the Pico's end the
#    VBAT_SW strip (v 52.3..57) joins J2.2 and the F2.1 pins and runs to the channel north of U3P; J2 / J1 on the rear edge, F2 beside
#    them (the VBAT_SW copper wraps its 2 x 2 F2.1 pins from both sides), F1, K1 (r 180: NO toward C1) and C1 on the VMOT bar to U4P.1,
#    J3 on the right edge (PB5 fallback; B12a: a right-angle Micro-Fit, its plug entering from beyond the right edge). The Pico-side pull-ups / pull-downs, the right encoder shifters, TP2 and R22 sit under the Pico.
#  - under the carrier (u >= 28, <= 16.27 mm): the drivers (both DRV8874 carriers turned 180 deg in place: logic rows U3L / U4L toward
#    the Pico at u 34.7 / 34.0, VIN/OUT/P rows U3P / U4P toward J4 / J5 at u 47.4 / 46.7, pin 1 at the low-v end; VIN on the VMOT bars),
#    the left encoder shifters Q1 / R1 / Q2 / R3 under the IMU module (outside both driver interiors; the relay driver Q5 / D1 / R9
#    sits in the Pico pocket, u 14 - 15), the IMU (U2 at v 17.3 leaves the VBAT_SW corridor 4.97 mm between
#    U2.8 and U3P.7), J10 / J11 on the side edges, H5 / H6 clamp holes, the status lamps and four test pads (see below), J14 (DNP).
#  - tongue: J4 / J5 right-angle toward their motor notches, plugs lying on the board (PB6); U5A (VBAT_SW, reached by the corridor under
#    J4's pins) and U5B at the north end of the front edge, then J8, J9; J15 (DNP) and LED3 / R23 between H3 / H4 and the front parts.
PLACEMENT = {
    # --- band: rear-edge XH row (1.35 mm gaps: a silk label above each) and the Pico
    'J13': (3.0, 7.05, 90), 'J6': (3.0, 24.39, 90), 'J7': (3.0, 41.73, 90), 'J12': (3.0, 51.58, 90),
    'U1A': (8.8, 2.75, 0), 'U1B': (8.8 + PICO_ROW_SPACING, 2.75, 0),
    # --- band: battery block (F2 between J2 and F1 so the VBAT_SW copper wraps its F2.1 pins), relay, C1, Jetson 5 V
    # B4: F1 0.15 mm toward +v (zone-driven nudge): the VBAT_SW F.Cu neck between U1B.20 and F1.2 4.79 -> 4.94 mm >= 4.84 mm (IPC 7.5 A,
    # 1 oz); its courtyard keeps 0.03 mm from C1's
    # B12a: J1 is the XT30UPB-M (pins 5 mm apart, body 10.2 mm along v): pin 2 (BAT+) at v 68.0 keeps the hold-down finger's 2.2 mm gap to
    # J2's body (HD_MIN_GAP, cad/build_robot.py), so K1 moves 0.5 mm toward +v to keep its courtyard off J1's (0.15 mm between them);
    # u 4.15 (0.2 mm nearer the rear edge than the VH) leaves room for J1's label beyond its pin marks, clear of F2 (PIN_MARK_AT).
    # J3 is the Micro-Fit right-angle header: pins at v 90.5 put its mating face 0.58 mm inside the right edge; u 24.1 keeps its courtyard
    # between K1's (u 17.14) and the carrier line (u 28).
    'J2': (4.35, 61.11, 90), 'J1': (4.15, 73.0, 90), 'F2': (15.98, 57.3, 270), 'F1': (23.19, 71.45, 90),
    'K1': (8.64, 96.45, 180), 'C1': (22.58, 84.33, 90), 'J3': (24.1, 90.5, 180),     # B4: C1 1.0 mm toward +v (BAT+ 4.9 mm south of F1.1)
    # --- under the Pico (SMD, reflowed before the module goes in; a silk label above each): pull-ups / pull-downs beside their GPIO
    # pins in two columns (u 14.0 by U1A, u 21.4 by U1B), the right encoder shifters at the top of the U1B column, TP2, R22 (5 V lamp)
    'R2': (14.0, 4.5, 0), 'R19': (14.0, 8.0, 0), 'R20': (14.0, 11.5, 0), 'R4': (14.0, 15.0, 0), 'R17': (14.0, 25.0, 0),
    'R18': (14.0, 28.5, 0), 'R10': (14.0, 32.0, 0), 'R6': (14.0, 38.0, 0), 'R8': (14.0, 41.5, 0), 'TP2': (34.0, 90.0, 0),
    'Q3': (21.4, 5.0, 0), 'R5': (21.4, 10.4, 0), 'Q4': (21.4, 15.0, 0), 'R11': (21.4, 19.5, 0), 'R12': (21.4, 23.0, 0),
    'C2': (21.4, 26.5, 0), 'R7': (21.4, 30.0, 0), 'R22': (21.4, 33.5, 0), 'R16': (21.4, 37.5, 0), 'R15': (21.4, 41.0, 0), 'R14': (21.4, 44.5, 0),
    'R13': (21.4, 48.0, 0),
    # PB17: TP2 (VMOT test pad) from (47, 94) to beside the VMOT bar south of U4L.7: its tap had run a 0.8 mm In2 diagonal across
    # the south of DRV_R, the only way out for BUMP_R / COIL_P
    # --- under the carrier: drivers (rows along v; PB15 / PB16: both turned, r 0 - logic rows west, VIN/OUT rows east, pin 1 at the low-v end)
    # B5: DRV_L (U3P / U3L) 0.7 mm toward +u (zone-driven nudge): the VBAT_SW F.Cu
    # channel between U1B and U3P's motor pins widens 4.27 -> 4.97 mm, so F.Cu alone carries VBAT_SW's 7.5 A (IPC 4.84 mm) and its In2
    # copy shrinks to a helper in the band - In2 then opens under the whole VBAT_SW channel / arm for the Pico -> driver crossings
    # PB15 / PB16: both carriers turned 180 deg in place (same rectangles): logic rows U3L / U4L on the west (u 34.7 / 34.0, pin 1 at the
    # north end), VIN/OUT rows U3P / U4P on the east (u 47.4 / 46.7) - OUT1 / OUT2 face J4 / J5, IN1 / IN2 / SLEEP / 3V3 face the Pico.
    # DRV_L's VIN is 47 mm from C1, so it gets its own bulk capacitor C3 (PB16).
    'U3L': (34.7, 42.0, 0), 'U3P': (47.4, 42.0, 0), 'U4L': (34.0, 67.0, 0), 'U4P': (46.7, 67.0, 0),
    # PB16: C3, DRV_L's bulk capacitor (100 uF V-chip, 5.8 mm since PB20: under the IMU module's body, below MODULE_CLEARANCE), 10.3 mm from
    # U3P.1; its + pad in a VMOT tab off the bar north of DRV_L, its GND pad to In1 per LP12 (the VBAT_SW arm notched round both)
    'C3': (40.3, 34.6, 90),
    # B5 / PB13: the left encoder shifters sit on the free F.Cu under the IMU module (north of the VBAT_SW arm), out of DRV_L's interior,
    # so both driver interiors stay open for the Pico -> driver links
    'Q1': (37.5, 16.0, 0), 'R1': (42.0, 16.0, 90), 'Q2': (38.5, 21.0, 0), 'R3': (45.0, 18.25, 90),
    # B5 / PB13: the relay driver (coil sense Q5 + R9, flyback D1) leaves DRV_R's interior for the Pico pocket, beside MOTOR_ON_N (U1A.12,
    # R10) and on the way of COIL_P to J13; K1's own courtyard and the VMOT / F2_OUT / J3 copper leave no SMD room at the coil itself
    'Q5': (14.0, 46.0, 0), 'D1': (15.0, 20.5, 180), 'R9': (14.0, 35.0, 0),
    # LIO-226: R26 (ARM pull-up) at the foot of the U1A pull-up column, under Q5 beside U1A.19 (ARM): the column holds no free slot at the
    # label pitch, so the VBAT_SW F.Cu strip under the Pico's south end gets a notch for it (gen_power_zones). v 49.65: 1.0 mm off Q5's
    # courtyard, and the strip below it to J2.2 / F2.1's relief rings keeps 4.93 mm (>= 4.84 mm, 7.5 A on 1 oz)
    'R26': (13.2, 49.65, 0),
    # IMU (U2 at v 17.3: the VBAT_SW corridor passes U3P's last pin), bumpers on the side edges
    'U2': (31.2, 17.3, 0), 'J10': (39.8, 2.9, 0), 'J11': (42.4, 97.1, 180),
    # status lamps and test pads under the carrier (no room in the band; listed for BRINGUP): LED2 north of the IMU, LED1 / R21 and TP1 /
    # TP4 at the north edge (PB15b), TP5 under the IMU module, TP3 east of J4 (PB16), TP2 by the VMOT bar south of DRV_R (PB17)
    # B5 / PB13: LED3 / R23 (the VMOT lamp) to the VMOT side, south of DRV_R beside the VMOT bar over U4P.1: its tap no longer crosses the
    # gap between the drivers
    'LED2': (40.25, 10.5, 90), 'LED3': (40.5, 86.0, 90), 'R23': (37.5, 86.0, 90),
    # B5 / PB13: TP1 / TP4 to the north edge (room under the IMU module for the shifters), TP5 stays; PB16: TP3 east of J4 over the 5V
    # regulator box (C3 took its place north of DRV_L), its tap one PowerTap via into the 5V B.Cu box
    'TP4': (31.0, 5.0, 0), 'TP1': (49.5, 6.5, 0), 'TP5': (44.2, 22.6, 0), 'TP3': (61.0, 39.5, 0),
    # B4: J14 1.0 mm toward +u (zone-driven nudge): the 5V B.Cu strip passes between its square pin 1 and U4P.7 with 3.9 mm (was 3.1)
    # B5 (controller: the DNP spare rows move freely within the B3 rules): J14 out of the gap between the drivers - the right motor pair's
    # only way round U4L's row - to the tongue between H3 and H4 (2 mm SMD-to-THT pad gap to LED3 / R23, clear of both washers)
    # PB15b: R21 (LED1's series resistor) to the VBAT_SW source by TP1 on the north edge (TP1 2.5 mm over for the silk labels): R21.1
    # joins TP1 with a short tap, R21.2 - LED1 is a Signal-class link (a resistor-limited LED needs no 0.8 mm feed). With LED1 left
    # south of DRV_R that link found no path even routed first (run 23), so LED1 follows to the north edge too (PB13a: LEDs move within
    # the B3 rules), 12 mm from R21, turned 270 so its LP12 via clears the Pico-side 5V tap's In2 diagonal.
    'LED1': (35.75, 4.0, 270), 'R21': (47.25, 2.25, 0), 'J14': (59.3, 46.4, 0),
    # LIO-226: the CS RC filters beside the drivers' CS pins (U3P.7 / U4P.7), not under the Pico by U1B.9 / U1B.7: both Pico columns
    # are full at the label pitch (no legal 0805 spot under the Pico or between U1B and U2), and here they need no part moved. The ADC
    # sees the same RC, the series R still guards the pin (all of L_CS_ADC lies behind it) and the long run to the Pico carries the
    # filtered level, not the 20 kHz chopped one. R24 / C4 east of U3P, in the pocket between its pins, J14 and the H3 / H4 washers;
    # R25 / C5 inside DRV_R by U4P.7 (east of U4P the J5 plug's lane leaves 1.15 mm), C5's GND via east of the 5V B.Cu strip.
    'R24': (51.5, 51.5, 90), 'C4': (53.0, 47.25, 0), 'R25': (42.95, 81.49, 90), 'C5': (42.25, 77.5, 0),
    # LIO-226: D2 (DNP VSYS diode) at the north end of DRV_R's interior, under the carrier module (2.7 mm < MODULE_CLEARANCE: unplug
    # DRV_R to fit it), anode over the 5V B.Cu strip, VSYS run to the Pico's pin 39 (U1B.2). No legal SMA spot lies within 25 mm of
    # U1B.2, and the tongue's free strip is J4's motor plug and cable band; DRV_R's interior is the only one left.
    'D2': (40.5, 66.0, 0),
    # --- tongue: motor headers (pins toward the notches), P03 terminals and front cliffs on the front edge, DNP J15
    'J4': (48.2, 35.2, 90), 'J5': (65.21, 64.8, 270),
    # B3 fix 1: the front edge keeps the motor-cable bands clear (v 15..21 / 79..85, MOTOR_CABLE_EXIT + MOTOR_CABLE_W)
    'U5A': (68.5, 29.4, 90), 'U5B': (68.5, 40.7, 90), 'J8': (71.05, 48.15, 270), 'J9': (71.05, 65.8, 270),
    'J15': (64.9, 47.3, 0),            # B5: 3.1 mm +u, 1.6 mm +v (DNP): room for J14 beside it
    # --- mounting: M3 holes at board_outline holes_uv (B2b), the clamp holes at clamp_holes_uv (PB3)
    'H1': (47.0, 12.0, 0), 'H2': (47.0, 88.0, 0), 'H3': (55.0, 41.5, 0), 'H4': (55.0, 58.5, 0), 'H5': (33.5, 10.0, 0), 'H6': (33.5, 96.0, 0),
}

# Power zones (ruling R4, bay B4: 4 layers), board (u, v) polygons: POWER_ZONES entries dict(net, layers, poly[, trunk][, spoke]),
# generated by pcb/gen_power_zones.py (lanes less octagon notches; edit the generator, not the block). The layers are
# F.Cu / In2.Cu / B.Cu (copper.STACKUP order); In1.Cu is the solid GND plane (the GND trunk) and B.Cu also carries a GND fill under these
# zones (build_pcb, priority 0: one main piece plus the small islands the power pours cut off round a GND pad or via; the In1 plane is the
# GND trunk, the B.Cu fill extra copper stitched to it by the LP12 / STITCH_VIAS vias and the GND pins). A trunk entry (the default) joins every
# POWER_TRUNK pad of its net - every pad of a 4-pad fuse contact - on each of its layers with >= 3 mm of copper (zone_check) and its width
# sets the net's IPC-2221 capacity (B6, copper.py); trunk=False marks a helper pour that does not reach every trunk pad. Polygons keep
# >= 0.4 mm off other nets' pads (octagon notches round the round pads), 0.5 mm off the other zones of their layer, out of the washer /
# clamp keep-outs, and a trunk polygon >= 0.8 mm from the LP12 GND vias' centres. Why these layers: F2 and F1 are solid blocks of 2.2 mm
# pads plus an NPTH peg between their pin pairs, so BAT+, F1_OUT and F2_OUT cannot all cross one layer (F.Cu also holds VBAT_SW's
# J2.2 - F2.1 link); every 2 x 2 contact needs copper beside each pad column, so F1's east pads take the channel on B.Cu; the H3 / H4
# washers and J14 close every >= 3 mm 5V route through the tongue; R16 keeps every zone 1.0 mm off the motor pins (U3P.3 / U3P.4 in the
# channel).
#  - VBAT_SW, F.Cu trunk + an In2.Cu helper round J2.2 (B5: the In2 copy cut back to a helper) (7.5 A: 4.84 mm on 1 oz alone): J2.2, round the F2.1 pins (copper west of F2 and in the
#    4.1 mm F2 / F1 gap), the strip over F2 and F1 (to v 49.2 under the Pico's south end, between TP2 and R13), north up the channel between
#    U1B and U3P (4.27 mm at U3P's motor pins), past U3P.7 and U2.8, the column beside the IMU socket, east under J4's pins (v 24 - 33.9)
#    to U5A.1. F.Cu 4.1 - 4.3 mm (6.7 - 6.9 A) + In2 the same (2 A): about 8.7 A.
#  - VMOT, F.Cu trunk + an In2.Cu helper under the spine and bars (5 A: 2.77 mm): the spine west of the P rows (u 28 - 32.75), the bar over U3P.1 (u 31.5 - 37: copper on
#    three sides of the pin), the bar east over U4P.1, the bar west through C1.1 to K1's NO pin.
#  - 5V, B.Cu trunk (4.5 A: 2.4 mm): U5B.1 north between J4.6 and U5B.2, west north of J4's pins (v 25.5 - 31.5) to u 41.5, south through
#    DRV_L's interior, west of J14 through the gap between the drivers, DRV_R's interior, out between U4P.1 and U4L.1 and west to J3.1;
#    octagon notches round the GND vias of Q1 / R1 / Q2 / R3 / Q5 / D1 / R9 leave >= 3.1 mm (5.5 A). The In2 helper round U5B.1 feeds the
#    J4 / J5 / J15 5 V taps.
#  - BAT+, B.Cu trunk: J1.2 (PB20: J1's north pin; J1.1 is GND) east between J1 and F2, south of F2, into the F2 / F1 gap, south of F1.1
#    (C1 1 mm further +v) and east of F1 in the channel (v 64.1 - 80); PB20 dropped its In2 copy (B.Cu alone 8.6 A). F1_OUT, B.Cu trunk: J2.1 north between J2.2 and F2.1 (5.1 mm past U1A.20), over F2 (to v 48.9 under the
#    Pico), F1.2 and east of F1 in the channel (v 52.3 - 63.6); In2 copy (fix 1) J2.1 - between J1 and F2 - under F2 - the F2 / F1 gap -
#    F1.2, stitched to its B.Cu trunk (STITCH_VIAS); both nets have 1.5 mm spokes (PB12). LIO-236: F1_OUT's B.Cu also rounds J2.2 (J2.1's
#    west pocket made live) and an F.Cu helper west and south of J2.1, stitched to the B.Cu trunk by three vias, takes J2.1's W and S spokes. F2_OUT, F.Cu trunk: F2.2 (copper west of F2 and in the F2 / F1 gap), south
#    between J1 and K1 to K1's COM pin.
#  - 3V3, In2 pour: under the Pico (u 10.2 - 25.2, v 1 - 34) with a lobe over the Pico's 3V3 pin (U1B.5), round the shifters Q3 / Q4 and
#    the pull-ups; U2, Q1 / Q2, the cliff connectors and the PMODE pins take tracks.
# Left free for B5's crossings out of the Pico corner: B.Cu in the channel north of F1 (v 36 - 52.3) and the B.Cu pocket north of DRV_L
# (u 35.3 - 41.5, v 31.5 - 41) for via hops to In2 / F.Cu; In2 inside both driver interiors and the tongue; F.Cu round the SMD parts.
# --- BEGIN generated by gen_power_zones.py (do not edit by hand) ---
POWER_ZONES = [
    dict(net='VBAT_SW', layers=('F.Cu',), spoke=1.5,
         poly=[(66.6, 26.1), (71.2, 26.1), (71.2, 33.9), (52.6, 33.9), (52.6, 33.3), (41.55, 33.3), (41.55, 28.95), (39.05, 28.95),
               (39.05, 31.2), (38.6, 31.2), (38.6, 41), (35.54, 41), (35.24, 40.7), (34.16, 40.7), (33.86, 41), (33.45, 41),
               (33.45, 41.41), (33.4, 41.46), (33.4, 44), (33.45, 44), (33.45, 55.9), (31, 55.9), (31, 58.5), (27.6, 58.5), (27.6, 58.3),
               (27.15, 58.3), (27.12, 58.17), (27.04, 57.98), (26.94, 57.81), (26.82, 57.66), (26.68, 57.52), (26.53, 57.4),
               (26.36, 57.3), (26.17, 57.22), (25.98, 57.17), (25.79, 57.14), (25.59, 57.14), (25.4, 57.17), (25.21, 57.22),
               (25.02, 57.3), (24.85, 57.4), (24.7, 57.52), (24.56, 57.66), (24.44, 57.81), (24.32, 57.66), (24.18, 57.52),
               (24.03, 57.4), (23.86, 57.3), (23.67, 57.22), (23.48, 57.17), (23.29, 57.14), (23.09, 57.14), (22.9, 57.17),
               (22.71, 57.22), (22.52, 57.3), (22.35, 57.4), (22.2, 57.52), (22.06, 57.66), (21.94, 57.81), (21.84, 57.98),
               (21.76, 58.17), (21.73, 58.3), (21.69, 58.3), (21.69, 58.51), (21.68, 58.55), (21.68, 58.75), (21.69, 58.79),
               (21.69, 62.1), (6.1, 62.1), (6.1, 59.36), (0.6, 59.36), (0.6, 53.4), (2.6, 53.4), (2.6, 52.9), (4.5, 52.9), (4.5, 52.3),
               (7.6, 52.3), (7.6, 51.65), (8.26, 52.31), (9.34, 52.31), (10.1, 51.55), (10.1, 50.7), (15.2, 50.7), (15.2, 49.2),
               (19.5, 49.2), (19.5, 51.6), (25.47, 51.6), (25.53, 51.71), (25.63, 51.84), (25.75, 51.96), (25.88, 52.06), (26.02, 52.14),
               (26.18, 52.2), (26.33, 52.24), (26.5, 52.27), (26.66, 52.27), (26.83, 52.24), (26.98, 52.2), (27.14, 52.14),
               (27.28, 52.06), (27.41, 51.96), (27.53, 51.84), (27.63, 51.71), (27.69, 51.6), (27.85, 51.6), (27.85, 36), (30.35, 36),
               (30.37, 36.03), (30.5, 36.13), (30.64, 36.21), (30.8, 36.27), (30.95, 36.31), (31.12, 36.34), (31.28, 36.34),
               (31.45, 36.31), (31.6, 36.27), (31.76, 36.21), (31.9, 36.13), (32.03, 36.03), (32.05, 36), (32.45, 36), (32.45, 24),
               (66.6, 24)]),
    dict(net='VBAT_SW', layers=('In2.Cu',), trunk=False,
         poly=[(8, 59.36), (2.6, 59.36), (2.6, 52.9), (8, 52.9)]),
    dict(net='VMOT', layers=('F.Cu',), spoke=1.5,
         poly=[(42.5, 38.6), (46.5, 38.6), (46.5, 39.4), (48.3, 39.4), (48.3, 43), (42.3, 43), (42.3, 59), (48, 59), (48, 63),
               (48.5, 63), (48.5, 67.9), (44.9, 67.9), (44.9, 63), (32.75, 63), (32.75, 81.5), (36, 81.5), (36, 86.3), (28, 86.3),
               (28, 88.9), (12.6, 88.9), (12.6, 80.75), (28, 80.75), (28, 59), (31.5, 59), (31.5, 56.4), (36, 56.4), (36, 41.4),
               (39.3, 41.4), (39.3, 38.6), (39, 38.6), (39, 35), (42.5, 35)]),
    dict(net='VMOT', layers=('In2.Cu',), trunk=False, spoke=1.5,
         poly=[(32.75, 81.5), (36, 81.5), (36, 86.3), (28, 86.3), (28, 88.9), (12.6, 88.9), (12.6, 80.75), (28, 80.75), (28, 59),
               (32.75, 59)]),
    dict(net='5V', layers=('B.Cu',),
         poly=[(66.8, 38.2), (71, 38.2), (71, 44.5), (62.2, 44.5), (62.2, 31.5), (45.4, 31.5), (45.4, 41), (46.1, 41), (46.1, 43.35),
               (43.03, 43.35), (42.5, 43.88), (42.5, 44.62), (43.03, 45.15), (46.1, 45.15), (46.1, 61.1), (42.35, 61.1), (42.35, 83.6),
               (41.1, 83.6), (41.1, 86.9), (33.25, 86.9), (33.25, 88.6), (28.9, 88.6), (28.9, 92.8), (22.3, 92.8), (22.3, 86), (24, 86),
               (24, 83.6), (37.9, 83.6), (37.9, 78), (38.6, 78), (38.6, 70.8), (35.25, 70.8), (35.25, 68.3), (38.6, 68.3), (38.6, 58.5),
               (37.5, 58.5), (37.5, 41), (39, 41), (39, 33), (42, 33), (42, 31.5), (41.5, 31.5), (41.5, 25.5), (66.8, 25.5)]),
    dict(net='5V', layers=('In2.Cu',), trunk=False,
         poly=[(69.6, 62.5), (66.15, 62.5), (66.15, 38), (69.6, 38)]),
    dict(net='BAT+', layers=('B.Cu',), spoke=1.5,
         poly=[(7.5, 64.4), (11.98, 64.4), (11.98, 71.6), (17.5, 71.6), (17.5, 66.2), (27.3, 66.2), (27.3, 64.1), (32.75, 64.1),
               (32.75, 80), (24, 80), (24, 77.9), (21.15, 77.9), (21.15, 80), (6.1, 80), (6.1, 71.1), (0.6, 71.1), (0.6, 65), (2, 65),
               (2, 63.5), (7.5, 63.5)]),
    dict(net='F1_OUT', layers=('B.Cu',), spoke=1.5,
         poly=[(25.28, 51.55), (25.3, 51.57), (25.3, 53.9), (33.45, 53.9), (33.45, 58), (32.75, 58), (32.75, 63.6), (17.53, 63.6),
               (17.53, 56.66), (17.5, 56.63), (17.5, 56.5), (17.37, 56.5), (16.62, 55.75), (15.34, 55.75), (14.73, 56.36),
               (14.12, 55.75), (12.84, 55.75), (12.09, 56.5), (11.95, 56.5), (11.95, 56.64), (11.93, 56.66), (11.93, 63.9), (8, 63.9),
               (8, 63), (0.6, 63), (0.6, 53.4), (6.15, 53.4), (6.15, 50.5), (7.5, 50.5), (7.5, 51.55), (8.26, 52.31), (9.34, 52.31),
               (10.1, 51.55), (10.1, 48.9), (25.28, 48.9)]),
    dict(net='F1_OUT', layers=('F.Cu',), trunk=False, spoke=1.5,
         poly=[(5.6, 62.6), (11.9, 62.6), (11.9, 64.1), (5.6, 64.1), (5.6, 65.6), (0.6, 65.6), (0.6, 59.86), (5.6, 59.86)]),
    dict(net='F1_OUT', layers=('In2.Cu',), trunk=False, spoke=1.5,
         poly=[(11.98, 71.6), (17.48, 71.6), (17.48, 62.6), (20.5, 62.6), (20.5, 60.6), (22.99, 60.6), (22.99, 65), (21.69, 65),
               (21.69, 75), (6.1, 75), (6.1, 65.86), (0.6, 65.86), (0.6, 59.86), (11.98, 59.86)]),
    dict(net='F2_OUT', layers=('F.Cu',), spoke=1.5,
         poly=[(21.6, 76), (12.1, 76), (12.1, 98.2), (6.3, 98.2), (6.3, 86.5), (7.3, 86.5), (7.3, 79), (7.6, 79), (7.6, 64.6),
               (21.6, 64.6)]),
    dict(net='3V3', layers=('In2.Cu',), trunk=False,
         poly=[(16.6, 11.62), (27.3, 11.62), (27.3, 14.2), (16.6, 14.2), (16.6, 34), (13.4, 34), (13.4, 1), (16.6, 1)]),
]
GND_PATCHES = [
    [(7.1, 78.5), (6.8, 78.5), (6.8, 86), (5.8, 86), (5.8, 98), (0.6, 98), (0.6, 69.4), (7.1, 69.4)],
    [(47.4, 45.2), (42.7, 45.2), (42.7, 43.65), (47.4, 43.65)],
]
STITCH_VIAS = [('F1_OUT', (1.6, 60.4)), ('F1_OUT', (19, 63.2)), ('F1_OUT', (20.6, 63.2)), ('F1_OUT', (8.5, 63.25)),
               ('F1_OUT', (10, 63.25)), ('F1_OUT', (11.5, 63.25)), ('GND', (1.2, 84.7)), ('GND', (4.6, 84.7)), ('GND', (1.2, 86.9)),
               ('GND', (4.6, 86.9)), ('GND', (1.2, 89.1)), ('GND', (4.6, 89.1)), ('GND', (43.4, 44.25)), ('GND', (45.6, 44.25)),
               ('GND', (18.6, 94)), ('GND', (18.8, 96.6))]
# --- END generated by gen_power_zones.py ---
# GND_PATCHES: F.Cu GND copper (ruling PB12, B4 fix 1) round the GND trunk pins whose In1 spokes alone cannot carry their load (the battery
# GND pin - J1.1 since PB20, its patch down the west edge to K1.5 - and U3P.2), stitched to In1 by STITCH_VIAS; build_pcb makes one F.Cu GND
# zone per patch (GND_PATCH_<k>).
# STITCH_VIAS: (net, (u, v)) through vias of the net class's size (net_via), placed by build_pcb and kept by route.py as pre-routes: the GND
# patches and B.Cu GND pockets to In1, the F1_OUT In2 copy to its B.Cu trunk at both ends.


def zone_layers(net, trunk_only=True):
    """The layers (copper.STACKUP names, in stack order) the net has POWER_ZONES entries on - its trunk entries only, unless
    trunk_only is False."""
    on = {L for z in POWER_ZONES if z['net'] == net and (z.get('trunk', True) or not trunk_only) for L in z['layers']}
    return tuple(L for L in copper.STACKUP if L in on)


def zones_on(layer):
    """[(net, poly)] of the POWER_ZONES entries poured on `layer` (helper pours included)."""
    return [(z['net'], z['poly']) for z in POWER_ZONES if layer in z['layers']]


# PB18: a pre-route may be narrower than its net class only with a `narrow` entry - a scripted feed of a probe pad (TP*) or a lamp
# (ruling PB15b), which carries no load - giving the reason and the current it carries (for B6). NARROW_TRACKS lists them.
TP1_FEED = dict(current_a=TAP_LOAD_A[('TP1', '1')] + TAP_LOAD_A[('R21', '1')],     # B6: the lamp at VBAT_MAX, 2.1 mA
                reason='TP1 probe feed (no load) + R21 lamp feed (LED1, 3.3k from VBAT_SW: ~2 mA); 0.3 mm between '
                       "H1's washer keep-out and the board edge")
# Scripted pre-routes (route.py adds them locked before Freerouting): the four motor tracks (B5, below), 5V taps, the inter-driver links.
PREROUTES = [
    # B5: the motor pairs (Motor class 1.0 mm, 1.0 mm clearance R16; outer layers only, R4: 1.0 mm on 0.5 oz In2 carries 0.7 A, not 2.1).
    # Freerouting cannot thread them (one 3 mm corridor per side and per layer), so they are scripted and locked. A layer change uses
    # three Motor vias (one 0.8 / 0.4 mm via carries about 0.94 A at 10 degC, copper.via_current_a).
    # PB16 (DRV_L turned): the pair leaves U3P east and runs north in the 1 mm corridor between U3P's row and H3's washer, stacked: L_M1
    # on F.Cu, L_M2 on B.Cu, each TH pin to TH pin with no via, into J4.1 / J4.2 from the south.
    dict(net='L_M1', layer='F.Cu', width=1.0, pts=[(47.4, 47.08), (49.85, 47.08), (49.85, 37.8), (48.2, 37.8), (48.2, 35.2)]),
    dict(net='L_M2', layer='B.Cu', width=1.0, pts=[(47.4, 49.62), (49.85, 49.62), (49.85, 37.6), (50.74, 36.71), (50.74, 35.2)]),
    # PB15 (DRV_R turned): R_M1 on F.Cu, R_M2 on B.Cu, each TH pin to TH pin with no via - they cross on different layers (the pair's
    # order flips between U4P's row and J5's): east from U4P.3 / U4P.4, north into J5.1 / J5.2 from the south.
    dict(net='R_M1', layer='F.Cu', width=1.0, pts=[(46.7, 72.08), (63.4, 72.08), (65.21, 70.27), (65.21, 64.8)]),
    dict(net='R_M2', layer='B.Cu', width=1.0, pts=[(46.7, 74.62), (60.9, 74.62), (62.67, 72.85), (62.67, 64.8)]),
    # PB16: TP3 (5V test pad) east of J4 over the 5V regulator box (C3 took its place north of DRV_L); its tap: one PowerTap via into
    # the 5V B.Cu box.
    dict(net='5V', layer='F.Cu', width=0.8, pts=[(61.0, 39.5), (62.9, 39.5)]),
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(62.9, 39.5), (63.4, 39.5)]),
    # LIO-232: D2.2's 5V tap (the DNP VSYS diode's anode, 0.15 mm off the 5V B.Cu strip through DRV_R) the same way: one PowerTap via
    # into the strip. Left to the router it came from J3 by a 32 mm F.Cu chain whose 18 mm leg walled off the R23 - LED3 gap.
    dict(net='5V', layer='F.Cu', width=0.8, pts=[(42.5, 66.0), (41.6, 64.6)]),
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(41.6, 64.6), (41.1, 64.6)]),
    # LIO-232: VSYS (D2.K -> the Pico's VSYS pin U1B.2; PICO_VSYS_A with D2 fitted), 0.25 mm (0.27 A on 0.5 oz In2 at 10 degC, where
    # 0.15 mm carries 0.18 A), two vias: F.Cu from D2.1 north of D2.2 and its 5V tap to a via east of the 5V B.Cu strip (south of the
    # VMOT bar); In2 up DRV_L's interior east of COIL_P, past U3L.1 and under the VBAT_SW F.Cu column to a via north of that zone's
    # band, south-west of the 3V3 In2 diagonal; B.Cu past U2's east side, round H5's keep-out and down the strip between U1B and U2 to
    # U1B.2. Left to the router it took stage F's 92 mm with four vias (Freerouting could not leave D2's pocket: its grown vias, VIA_GROW).
    dict(net='VSYS', layer='F.Cu', width=0.25, pts=[(38.5, 66.0), (40.8, 63.7), (43.6, 63.7)]),
    dict(net='VSYS', layer='In2.Cu', width=0.25, pts=[(43.6, 63.7), (39.6, 59.7), (39.6, 45.4), (36.0, 41.8), (34.0, 23.0)]),
    dict(net='VSYS', layer='B.Cu', width=0.25, pts=[(34.0, 23.0), (34.0, 16.6), (29.4, 12.0), (29.4, 8.11), (26.58, 5.29)]),
    # PB14: the Pico-side 5V taps (R22.1, J13.1) round the Pico's north end to the 5V B.Cu arm (run 9's DRC-clean route, its east end
    # cut short into the arm). Left to Freerouting they cut across the strip between U1B and U3P - the driver signals' only way in.
    dict(net='5V', layer='F.Cu', width=0.8, pts=[(20.488, 33.5), (19.25, 32.26), (19.25, 24.33)]),
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(19.25, 24.33), (16.86, 21.94), (16.86, 8.11), (11.92, 3.17)]),
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(16.86, 8.11), (17.74, 8.99), (17.87, 8.99)]),
    dict(net='5V', layer='In2.Cu', width=0.8, pts=[(11.92, 3.17), (9.95, 1.2), (7.59, 1.2), (5.46, 3.33), (5.46, 4.59), (3.0, 7.05)]),
    dict(net='5V', layer='In2.Cu', width=0.8, pts=[(17.87, 8.99), (18.42, 8.99), (24.29, 3.13), (24.29, 2.28), (25.37, 1.2), (30.66, 1.2),
                                                  (37.73, 8.26), (37.73, 16.66), (40.3, 23.3)]),
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(40.3, 23.3), (41.9, 25.9)]),
    # PB18: COIL_P (Q5.1 / R9.1 / K1.2 / J13), in the critical bundle: with Freerouting's fanout stage off (PB18b) stage A lost it in both
    # run 48 variants. Run 43 set-up 0's own route: F.Cu up the pull-up column to R9.1 and J13, In2 from a via by Q5 through U1B's .15/.16
    # and U3L's .2/.3 gaps, down DRV_L / DRV_R's interiors (u 38.18) and round the south to K1.2.
    # B12a: its tail follows K1.2 0.5 mm toward +v and runs at v 92.4, 0.8 mm clear of the Micro-Fit J3's pads and its 3 mm NPTH peg.
    dict(net='COIL_P', layer='F.Cu', width=0.15, pts=[(3.0, 4.55), (3.0, 5.627), (4.077, 5.627), (7.55, 9.1), (9.076, 9.1), (11.433, 11.457), (11.433, 15.771), (16.162, 20.5), (14.035, 22.627), (14.035, 34.053), (13.088, 35.0), (13.829, 35.741), (13.829, 42.515), (13.337, 43.007)]),
    dict(net='COIL_P', layer='F.Cu', width=0.15, pts=[(13.337, 43.007), (13.337, 44.776), (13.062, 45.05)]),
    dict(net='COIL_P', layer='F.Cu', width=0.15, pts=[(16.162, 20.5), (17.0, 20.5)]),
    dict(net='COIL_P', layer='In2.Cu', width=0.15, pts=[(13.337, 43.007), (16.599, 39.745), (28.332, 39.745), (34.397, 45.81), (35.009, 45.81), (38.178, 48.978), (38.178, 85.084), (30.862, 92.4), (17.667, 92.4), (15.617, 94.45), (14.64, 94.45)]),
    # PB18b: TP1's VBAT_SW feed (probe pad, and R21's lamp feed through it): the router fed it through the band north of DRV_L (F.Cu /
    # B.Cu u 33.4 - 34, In2 v 8), right where the front-right group crosses. Scripted east of H1 instead, down the main body's edge
    # (0.3 mm - the PB18 probe-feed exception, NARROW_TRACKS - where H1's 4.5 mm washer keep-out leaves 0.45 mm to the edge), a B.Cu hop
    # under XSHUT4's F.Cu and into the VBAT_SW F.Cu arm; R21.1 -> TP1 at full width.
    dict(net='VBAT_SW', layer='F.Cu', width=0.8, pts=[(46.34, 2.25), (46.34, 3.34), (49.5, 6.5)]),
    dict(net='VBAT_SW', layer='F.Cu', width=0.3, pts=[(49.5, 6.5), (51.75, 8.75), (51.75, 16.6)], narrow=TP1_FEED),
    dict(net='VBAT_SW', layer='B.Cu', width=0.3, pts=[(51.75, 16.6), (51.75, 19.6)], narrow=TP1_FEED),
    dict(net='VBAT_SW', layer='F.Cu', width=0.3, pts=[(51.75, 19.6), (51.75, 24.6)], narrow=TP1_FEED),
    # PB18: TP2's VMOT tap (test pad at (34.0, 90.0) under the VMOT bar): straight up on F.Cu into the bar at full PowerTap width - the
    # router had necked it to 0.6 mm.
    dict(net='VMOT', layer='F.Cu', width=0.8, pts=[(34.0, 90.0), (34.0, 85.8)]),
    # PB14: J4.4's 5V tap, straight down on B.Cu into the arm.
    dict(net='5V', layer='B.Cu', width=0.8, pts=[(55.82, 35.2), (55.82, 31.0)]),
    # PB15: J5.4's 5V tap, on In2 under J5's row (1.15 mm off the Motor pins J5.1 / J5.2) to the 5V In2 helper's corner; Freerouting
    # left it open in every run.
    dict(net='5V', layer='In2.Cu', width=0.8, pts=[(57.59, 64.8), (59.0, 63.0), (61.5, 62.4), (66.3, 62.4)]),
    # PB17: the two inter-driver links U3L.5 - U4L.5 (3V3) and U3L.6 - U4L.6 (DRV_SLEEP), open in every PB17 run: leaving U3L the pair's
    # order (5 north of 6) is the wrong way round for U4L, and the only free copper between the logic rows is B.Cu east of U3L (to the
    # 5V lane) and F.Cu / In2 east of U4L (south of the VMOT bar, v > 63.6). Both run south on B.Cu in that strip; DRV_SLEEP (inner)
    # drops to F.Cu below the VMOT bar into U4L.6, 3V3 (outer) to In2 into U4L.5 - they cross on different layers. The In2 strip
    # between U4L's row and 3V3 (2.4 mm) stays open for R_IN1 / R_IN2.
    dict(net='DRV_SLEEP', layer='B.Cu', width=0.15, pts=[(34.7, 54.7), (35.95, 55.95), (35.95, 63.9)]),
    dict(net='DRV_SLEEP', layer='F.Cu', width=0.15, pts=[(35.95, 63.9), (35.3, 64.55), (35.3, 79.7), (34.0, 79.7)]),
    dict(net='3V3', layer='B.Cu', width=0.15, pts=[(34.7, 52.16), (35.95, 52.16), (36.55, 52.76), (36.55, 63.15), (37.3, 63.9)]),
    dict(net='3V3', layer='In2.Cu', width=0.15, pts=[(37.3, 63.9), (37.3, 76.4), (36.54, 77.16), (34.0, 77.16)]),
    # PB17: that 3V3 pair's own link to the rest of 3V3 (open in 2 of run 37's 3 variants: U3L.5's west side is where DRV_SLEEP,
    # L_IN1 and L_IN2 cross): R8.2 (3V3 pull-up) -> In2 under the Pico -> B.Cu through U1B's .17 / .18 gap and U3L's .2 / .3 gap,
    # down the B.Cu strip east of U3L into the pre-route above (run 37 set-up 1's own route, DRC clean there).
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(14.91, 41.5), (15.12, 41.71), (15.12, 44.68)]),
    # ... and R8.2 / R6.2 (pull-ups below the 3V3 In2 strip's south end, v 34) into that strip: up the pad column on F.Cu to a via
    # under R6.2, then north on In2 (0.33 mm clear of R9's GND via) into the strip - run 39 left this short hop open in both variants.
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(14.91, 41.5), (14.91, 39.2)]),
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(14.91, 39.2), (14.91, 38.0)]),
    dict(net='3V3', layer='In2.Cu', width=0.15, pts=[(14.91, 39.2), (14.91, 33.6)]),
    # LIO-226: R26.2 (the ARM pull-up's 3V3 end, under Q5) up past Q5's drain pad into the 3V3 hop via above (15.12, 44.68); the first
    # route left it open.
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(14.112, 49.65), (14.112, 48.6), (16.2, 46.512), (16.2, 45.76), (15.12, 44.68)]),
    # PB17: DRV_SLEEP's own link (R17.1 -> U3L.6; open in every variant once 3V3 took the route above). West of U3L, L_IN1 (U1B.20 ->
    # U3L.3) and L_IN2 (U1B.19 -> U3L.4) cross each other, so a net from the far west into U3L.6 that passes U1B north of .19 crosses
    # both. Through U1B's .19 / .20 gap it crosses L_IN1 only: B.Cu under the Pico (over 3V3's In2 run), a via clear of the F1_OUT
    # B.Cu pour (u 22 - 25.3 from v 49), In2 through the gap and into U3L.6 from the west - L_IN1 then on B.Cu, L_IN2 on In2.
    dict(net='DRV_SLEEP', layer='F.Cu', width=0.15, pts=[(13.09, 25.0), (13.09, 26.3)]),
    dict(net='DRV_SLEEP', layer='B.Cu', width=0.15, pts=[(13.09, 26.3), (18.5, 31.71), (18.5, 42.8), (22.7, 47.0), (24.4, 47.0)]),
    dict(net='DRV_SLEEP', layer='In2.Cu', width=0.15, pts=[(24.4, 47.0), (24.4, 48.7), (25.44, 49.74), (27.72, 49.74), (32.68, 54.7),
                                                        (34.7, 54.7)]),
    # ... so L_IN1 (U1B.20 -> U3L.3, crossing it) is B.Cu, under the 3V3 run and over DRV_SLEEP's; scripted, as run 40 left it open
    # (L_IN2 then crosses it on In2).
    dict(net='L_IN1', layer='B.Cu', width=0.15, pts=[(26.58, 51.01), (28.6, 51.01), (32.53, 47.08), (34.7, 47.08)]),
    dict(net='L_IN2', layer='In2.Cu', width=0.15, pts=[(26.58, 48.47), (33.55, 48.47), (34.7, 49.62)]),     # run 41 left it open
    # PB17: R11.1's VBAT_SW tap (the VSENSE divider's top; open in 3 PB17 runs): south under the Pico on F.Cu (u 17.6, between the
    # pull-up columns) into the VBAT_SW F.Cu strip's north edge (v 49.2) - run 37 set-up 1's route without its In2 tail.
    dict(net='VBAT_SW', layer='F.Cu', width=0.8, pts=[(20.488, 19.5), (20.488, 20.1), (19.44, 21.15), (17.64, 22.95), (17.64, 39.62),
                                                     (17.34, 39.92), (17.34, 49.9)]),
    # PB17: the 3V3 trunk to the front-right sockets (TP4, the Pico's 3V3 pin U1B.5, U2.1, J14.1, J15.4, J8.1, J9.1): run 37's stage-A
    # 3V3 route (the same in all its variants); since the scripted links above, stage A left that group unjoined (runs 41, 42).
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(31.0, 5.0), (28.71, 7.29), (28.71, 10.89)]),
    dict(net='3V3', layer='In2.Cu', width=0.15, pts=[(28.71, 10.89), (26.69, 12.91)]),
    dict(net='3V3', layer='In2.Cu', width=0.15, pts=[(26.5, 12.57), (30.15, 16.22), (31.2, 16.22), (31.2, 17.3), (31.2, 18.38),
                                                    (31.57, 18.38), (41.16, 27.97), (50.38, 27.97), (57.2, 34.79), (57.2, 35.61),
                                                    (59.76, 38.17), (59.76, 44.86), (59.3, 45.32), (59.3, 46.4), (59.3, 47.48),
                                                    (59.43, 47.48), (60.38, 48.42), (60.38, 50.4), (64.9, 54.92)]),
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(71.05, 48.15), (67.28, 51.92), (67.28, 54.92), (67.28, 62.03), (71.05, 65.8)]),
    dict(net='3V3', layer='F.Cu', width=0.15, pts=[(64.9, 54.92), (67.28, 54.92)]),
    # PB17: SDA at the left-edge sockets (J6.3 - J7.3 on B.Cu, J6.3 - the Pico's U1A.4 on In2): XSHUT4's F.Cu run inside J6 / J7 leaves
    # their pins B.Cu / In2 only, and runs 44 / 45 left J6.3 - J7.3 open in all three variants. Run 43 set-up 0's own route.
    dict(net='SDA', layer='B.Cu', width=0.15, pts=[(3.0, 19.39), (4.612, 19.39), (5.963, 20.74), (5.963, 21.828), (8.475, 24.34),
                                                  (4.252, 28.563), (4.252, 34.671), (3.269, 35.653), (3.0, 35.653), (3.0, 36.73)]),
    dict(net='SDA', layer='In2.Cu', width=0.15, pts=[(3.0, 19.39), (3.0, 18.313), (4.077, 18.313), (8.178, 14.212), (9.13, 14.212),
                                                    (9.921, 13.421), (9.921, 11.491), (8.8, 10.37)]),
    # PB17: XSHUT4 (U1A.17 -> J9.5, rear-left to front-right; open in most PB17 runs) all on F.Cu round the board: up the left edge
    # inside J6 / J7, along the rear edge past U1A.1, between U1B's top pins and H5, east under J4's row's north side, down the front
    # edge outside J8 into J9.5 - run 36 set-up 0's own route.
    dict(net='XSHUT4', layer='F.Cu', width=0.15, pts=[(8.8, 43.39), (4.54, 39.13), (4.54, 14.28), (1.31, 11.06), (1.31, 4.07),
                                                     (3.71, 1.67), (19.7, 1.67), (20.9, 2.87), (21.94, 2.87), (25.45, 6.39),
                                                     (28.98, 6.39), (29.73, 5.64), (29.73, 4.52), (30.47, 3.77), (32.57, 3.77),
                                                     (38.09, 9.29), (38.09, 12.61), (41.43, 15.95), (42.45, 15.95), (43.79, 17.29),
                                                     (43.79, 17.57), (44.3, 18.08), (64.47, 18.08), (73.35, 26.96), (73.35, 72.69),
                                                     (71.32, 74.72), (71.05, 74.72), (71.05, 75.8)]),
    dict(net='3V3', layer='In2.Cu', width=0.15, pts=[(15.12, 44.68), (15.93, 45.49), (24.67, 45.49), (25.5, 44.66)]),
    dict(net='3V3', layer='B.Cu', width=0.15, pts=[(25.5, 44.66), (31.18, 44.66), (32.22, 45.7), (34.89, 45.7), (36.1, 46.91),
                                                  (36.1, 52.01), (35.95, 52.16)]),
]

# PB18: the only tracks allowed under their net class's width - the PREROUTES entries flagged `narrow` (one source: the pre-route itself
# carries the flag, reason and current). route.evaluate and ktest_route honour exactly these segments.
NARROW_TRACKS = [r for r in PREROUTES if 'narrow' in r]
def narrow_track_keys():
    """{(net, layer, width, frozenset({end, end}))} of the NARROW_TRACKS segments (1 um keys, as route.py compares tracks)."""
    r3 = lambda p: (round(p[0], 3), round(p[1], 3))
    return {(e['net'], e['layer'], round(e['width'], 3), frozenset({r3(a), r3(b)})) for e in NARROW_TRACKS for a, b in zip(e['pts'], e['pts'][1:])}

# GND vias of the top-side SMD GND pads (ruling LP12): one tented via per pad, 0.3 mm past the pad's outer end along its axis, so its copper
# overlaps the pad by 0.1 mm and its drill stays outside the paste opening - the pad joins the GND plane with no GND track. (u, v); every
# via keeps >= 0.5 mm from other nets' pads (R17a) and out of the power-trunk polygons. Generated for the B3 placement.
SMD_GND_VIAS = {
    'C2': (23.15, 26.5),
    'C3': (40.3, 29.85),           # PB16
    'C4': (54.75, 47.25),          # LIO-226
    'C5': (44.0, 77.5),            # LIO-226
    'D1': (11.45, 20.5),
    'Q1': (35.52, 16.95),
    'Q2': (36.52, 21.95),
    'Q3': (19.42, 5.95),
    'Q4': (19.42, 15.95),
    'Q5': (12.03, 46.95),
    'R1': (42, 14.28),
    'R3': (45, 16.52),
    'R5': (23.12, 10.4),
    'R7': (23.12, 30.0),
    'R9': (15.72, 35),
    'R12': (23.12, 23.0),
    'R13': (23.12, 48.0),
    'R14': (23.12, 44.5),
    'R15': (23.12, 41.0),
    'R16': (23.12, 37.5),
    'R17': (15.72, 25.0),
    'TP5': (42.9, 22.6),
    'LED1': (35.75, 2.27),         # PB15b
    'LED2': (40.25, 12.22),
    'LED3': (40.5, 87.72),
}

# Layer-change vias of the scripted pre-routes (where two PREROUTES segments of the net on different layers end, off a THT pin): the Pico-side 5V taps (PB14), the
# inter-driver links (PB17; signal vias 0.6 / 0.3).
HOP_VIAS = [
    ('5V', (62.9, 39.5)), ('5V', (19.25, 24.33)), ('5V', (11.92, 3.17)), ('5V', (17.87, 8.99)), ('5V', (40.3, 23.3)),   # PB14 / PB16: 5V taps
    ('5V', (41.6, 64.6)), ('VSYS', (43.6, 63.7)), ('VSYS', (34.0, 23.0)),       # LIO-232: D2.2's 5V tap, VSYS
    ('DRV_SLEEP', (35.95, 63.9)), ('3V3', (37.3, 63.9)),                       # PB17: the inter-driver links
    ('3V3', (15.12, 44.68)), ('3V3', (25.5, 44.66)), ('3V3', (14.91, 39.2)),   # PB17: the 3V3 pair's own link, R8.2 / R6.2 into the strip
    ('DRV_SLEEP', (13.09, 26.3)), ('DRV_SLEEP', (24.4, 47.0)),                 # PB17: DRV_SLEEP's west link
    ('3V3', (28.71, 10.89)),                                                   # PB17: the 3V3 trunk to the front-right sockets
    ('COIL_P', (13.337, 43.007)),                                              # PB18: COIL_P
    ('VBAT_SW', (51.75, 16.6)), ('VBAT_SW', (51.75, 19.6))]                    # PB18b: TP1's feed
# Via size per net class (diameter, drill) mm: the one table - make_schematic.py writes it to the .kicad_pro, route.py and ktest_route import it.
CLASS_VIA = {'Default': (0.6, 0.3), 'Power': (1.2, 0.6), 'PowerTap': (0.8, 0.4), 'Motor': (0.8, 0.4)}   # PB17: signal vias 0.6 / 0.3
# The GND stitching vias keep 0.8 / 0.4 (PB17: pinned at their size; GND is a Default-class net). LIO-226: the LP12 SMD-pad vias
# (SMD_GND_VIAS) drill 0.3 - at 0.4 their drill came within 0.095-0.105 mm of their pad's mask opening, at 0.3 the web is ~0.15 mm;
# they carry mA (C3's ripple share, the most, is under its 0.76 A barrel, ktest_power).
GND_VIA = (0.8, 0.4)
SMD_PAD_VIA = (0.8, 0.3)
def net_via(net, at=None):
    """(diameter, drill) of a via of `net`; at: its (u, v), so a GND via at an SMD_GND_VIAS spot is SMD_PAD_VIA."""
    if net == 'GND':
        lp12 = at is not None and (round(at[0], 3), round(at[1], 3)) in {(round(u, 3), round(v, 3)) for u, v in SMD_GND_VIAS.values()}
        return SMD_PAD_VIA if lp12 else GND_VIA
    return CLASS_VIA[next((name for name, c in NETCLASSES.items() if net in c['nets']), 'Default')]
PREROUTE_VIAS = [('GND', uv) for uv in SMD_GND_VIAS.values()] + STITCH_VIAS + HOP_VIAS
