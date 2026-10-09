# R1 Power & I/O Board Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Design an orderable 2-layer PCB that carries the robot's plug-in modules and replaces most of its wiring, and integrate it into the robot CAD.

**Architecture:** One Python data module (`pcb/board_spec.py`) is the single source of truth for parts, pins and nets. Generator scripts turn it into a KiCad schematic (net-label style, checked by KiCad's ERC), a placed PCB (KiCad `pcbnew` API), a routed PCB (Freerouting), fabrication files (`kicad-cli`), and a STEP model that the robot CAD (`cad/build_robot.py`) imports. Tests compare every generated artefact back to `board_spec.py`.

**Tech Stack:** KiCad 9 (`kicad-cli`, bundled Python with `pcbnew`), Freerouting 2.x (Java 21), CadQuery 2.8 (existing CAD venv), Python `unittest`.

**Spec:** `/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim/docs/superpowers/specs/2026-10-04-r1-power-io-board-design.md`

## Global Constraints

- Board: 2 layers, 2 oz copper, 1.6 mm FR-4, lead-free HASL; all parts through-hole (user hand-solders).
- Envelope: about 84 × 170 mm at robot x 14–98 mm, |y| ≤ 85 mm; PCB underside at robot z 39 (base top 33 + 6 mm standoffs); everything below the deck underside z 78.
- Mounting: four M3 screws into heat-set inserts in printed bosses on P_base.
- Connectors: JST-VH for power, JST-XH for signals, 1×6 2.54 mm male headers for the motor cables (vendor wire order).
- Fuses: F1 7.5 A main, F2 5 A motor branch, board-mount blade holders.
- Minimum width of any power path: 3 mm; signal traces and clearances ≥ 0.25 mm.
- Pico pin map exactly as spec section 4 (existing pins unchanged; new: GP18 nSLEEP, GP19 nFAULT, GP27/GP28 motor current).
- Pico powered only by its USB cable from the Jetson (no board feed to VBUS/VSYS).
- Downloads (KiCad, OpenJDK, Freerouting) only after the user approves each one (name, source, size). Nothing is ordered or paid for automatically.
- No git repository exists: every task ends with an **archive** step instead of a commit:
  `tar -C /Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs -czf "$SP/backup-original/pcb-task<N>.tgz" home-robot-r1/pcb home-robot-r1/cad/build_robot.py home-robot-r1/parts-and-connections.json`

Shell variables used throughout (define them in every shell):

```bash
R1=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-r1
SIM=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim
SP=/private/tmp/claude-501/-Users-liorknafo-Documents-Codex-2026-10-03-i-x20-outputs-home-robot-sim/030356d9-c2ab-4fc2-a131-e83b5b8d9fe0/scratchpad
CADPY=$SP/cadvenv/bin/python                      # CadQuery 2.8 venv (also runs the pure-Python tests)
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3   # KiCad's Python (pcbnew)
KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
```

Pure-Python tests run with `cd $R1 && $CADPY -m unittest discover -s pcb/tests -p 'test_*.py' -t .`; tests that import `pcbnew` are named `ktest_*.py` and run with `cd $R1 && $KPY -m unittest discover -s pcb/tests -p 'ktest_*.py' -t .`.

## File Structure

```
home-robot-r1/pcb/
  __init__.py            empty (package marker)
  tools.py               paths to kicad-cli, KiCad Python, libraries, Java, Freerouting
  board_spec.py          parts, footprints, symbols, pin maps, nets, placement, currents (single source of truth)
  sexpr.py               tiny S-expression reader/writer for KiCad files
  extract_outline.py     robot CAD -> board_outline.json (outline with notches, mounting holes) [CadQuery]
  board_outline.json     generated
  make_schematic.py      board_spec -> r1_power_io.kicad_sch (net-label schematic) + .kicad_pro
  build_pcb.py           board_spec + outline -> placed r1_power_io.kicad_pcb [KiCad Python]
  route.py               DSN -> Freerouting -> SES -> zones -> DRC [KiCad Python + Java]
  check_power.py         IPC-2221 width check of power nets [KiCad Python]
  fab.py                 Gerbers, drill, BOM, 1:1 PDF, STEP, ordering zip
  r1.pretty/             custom footprints (only if a standard one is missing)
  reference/             saved copies of vendor pinout pages used for board_spec
  fab/                   generated fabrication outputs
  ORDERING.md            how to order the board
  BRINGUP.md             bring-up and test guide
  tests/test_tools.py, test_board_spec.py, test_outline.py, test_schematic.py, test_bom.py
  tests/ktest_pcb.py, ktest_route.py, ktest_power.py, ktest_fab.py
home-robot-r1/cad/build_robot.py       modified: board + modules + base mounts in, old electronics out, harness rewired
home-robot-r1/parts-and-connections.json  modified via pcb/update_bom.py
```

---

### Task 1: Toolchain

**Files:**
- Create: `$R1/pcb/__init__.py`, `$R1/pcb/tools.py`, `$R1/pcb/tests/__init__.py`, `$R1/pcb/tests/test_tools.py`

**Interfaces:**
- Produces: `pcb.tools.KICAD_CLI`, `KICAD_PYTHON`, `SYMBOL_DIR` (Path), `FOOTPRINT_DIR` (Path), `JAVA`, `FREEROUTING` (str paths).

- [ ] **Step 1: Ask the user to approve the downloads** (one message, wait for an explicit yes):
  - KiCad 9 via Homebrew cask `kicad` (source: kicad.org builds, about 1.3 GB download, about 4 GB installed).
  - OpenJDK 21 via Homebrew `openjdk@21` (about 200 MB).
  - Freerouting 2.x jar from https://github.com/freerouting/freerouting/releases (about 40 MB) into `$R1/pcb/tools/`.

- [ ] **Step 2: Write the failing test** `$R1/pcb/tests/test_tools.py`:

```python
import subprocess, unittest
from pathlib import Path
from pcb import tools

class ToolTests(unittest.TestCase):
    def test_kicad_cli_is_version_9(self):
        out = subprocess.run([tools.KICAD_CLI, 'version'], capture_output=True, text=True).stdout.strip()
        self.assertTrue(out.startswith('9.'), out)
    def test_kicad_python_imports_pcbnew(self):
        r = subprocess.run([tools.KICAD_PYTHON, '-c', 'import pcbnew;print(pcbnew.Version())'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr[-400:]); self.assertTrue(r.stdout.strip().startswith('9.'), r.stdout)
    def test_libraries_present(self):
        self.assertTrue((tools.SYMBOL_DIR / 'Device.kicad_sym').is_file())
        self.assertTrue((tools.FOOTPRINT_DIR / 'Connector_JST.pretty').is_dir())
    def test_java_21_and_freerouting(self):
        r = subprocess.run([tools.JAVA, '-version'], capture_output=True, text=True)
        self.assertIn('version "21', r.stderr)
        self.assertGreater(Path(tools.FREEROUTING).stat().st_size, 1_000_000)

if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 3: Run it to verify it fails**

Run: `cd $R1 && touch pcb/__init__.py pcb/tests/__init__.py && $CADPY -m unittest pcb.tests.test_tools -v`
Expected: ERROR `ModuleNotFoundError: No module named 'pcb.tools'`

- [ ] **Step 4: Install (after approval) and write `pcb/tools.py`**

```bash
brew install --cask kicad
brew install openjdk@21
mkdir -p $R1/pcb/tools && cd $R1/pcb/tools
curl -L -o freerouting.jar "$(curl -s https://api.github.com/repos/freerouting/freerouting/releases/latest | $CADPY -c 'import json,sys;a=json.load(sys.stdin)["assets"];print([x["browser_download_url"] for x in a if x["name"].endswith(".jar")][0])')"
```

```python
"""Paths to the PCB toolchain (KiCad 9, Java 21, Freerouting)."""
from pathlib import Path
APP = Path('/Applications/KiCad/KiCad.app')
KICAD_CLI = str(APP / 'Contents/MacOS/kicad-cli')
KICAD_PYTHON = str(APP / 'Contents/Frameworks/Python.framework/Versions/Current/bin/python3')
SYMBOL_DIR = APP / 'Contents/SharedSupport/symbols'
FOOTPRINT_DIR = APP / 'Contents/SharedSupport/footprints'
JAVA = '/opt/homebrew/opt/openjdk@21/bin/java'
FREEROUTING = str(Path(__file__).parent / 'tools' / 'freerouting.jar')
PCB_DIR = Path(__file__).parent
```

If a path differs on this machine, find it with `mdfind -name kicad-cli` / `ls /Applications/KiCad/KiCad.app/Contents/SharedSupport` and correct the constant.

- [ ] **Step 5: Run it to verify it passes**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_tools -v`
Expected: 4 tests OK.

- [ ] **Step 6: Archive** (Global Constraints archive command, `<N>`=1).

---

### Task 2: Board source of truth (`board_spec.py`)

**Files:**
- Create: `$R1/pcb/board_spec.py`, `$R1/pcb/sexpr.py`, `$R1/pcb/reference/pololu-4035.txt`, `$R1/pcb/reference/pololu-2851.txt`, `$R1/pcb/tests/test_board_spec.py`
- Create (only if Step 4 finds no standard holder): `$R1/pcb/r1.pretty/Fuseholder_Blade_ATO_Keystone_3557-2.kicad_mod`

**Interfaces:**
- Consumes: `pcb.tools.SYMBOL_DIR`, `FOOTPRINT_DIR`.
- Produces (used by every later task):
  - `PARTS: dict[str, dict]` ref → `{sym: 'Lib:Name', fp: 'Lib:Name', value: str, mpn: str, h: float}` (`h` = height above the board, mm)
  - `NETS: dict[str, list[tuple[str, str]]]` net name → `[(ref, pin_number), ...]`
  - `NC: list[tuple[str, str]]` pins deliberately left unconnected
  - `PLACEMENT: dict[str, tuple[float, float, float]]` ref → `(u, v, rotation_deg)` in board mm
  - `CURRENT_A: dict[str, float]` power net → design current
  - `NETCLASS_POWER: list[str]`, `REMOVED_CAD_IDS: list[str]`
  - `robot_to_board(x, y) -> (u, v)`, `board_to_robot(u, v) -> (x, y)`, constants `BOARD_X0, BOARD_Y0, BOARD_W, BOARD_H, Z_PCB_BOTTOM, PCB_T, MAX_PART_H`
  - `pcb.sexpr.parse(text) -> list`, `pcb.sexpr.dump(node) -> str`, class `pcb.sexpr.Q(str)` (quoted string), `find(node, key) -> list`

- [ ] **Step 1: Write `pcb/sexpr.py`** (shared by schematic, netlist and library readers):

```python
"""Minimal KiCad S-expression reader/writer. Quoted strings come back as Q (a str subclass) so dump() re-quotes them."""
import re
class Q(str):
    pass
_TOKEN = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+')
def parse(text):
    stack = [[]]
    for t in _TOKEN.findall(text):
        if t == '(':
            stack.append([])
        elif t == ')':
            node = stack.pop(); stack[-1].append(node)
        elif t.startswith('"'):
            stack[-1].append(Q(t[1:-1].replace('\\"', '"').replace('\\\\', '\\')))
        else:
            stack[-1].append(t)
    return stack[0]
def dump(x):
    if isinstance(x, list):
        return '(' + ' '.join(dump(i) for i in x) + ')'
    if isinstance(x, Q):
        return '"' + str(x).replace('\\', '\\\\').replace('"', '\\"') + '"'
    return str(x)
def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]
```

- [ ] **Step 2: Write the failing test** `$R1/pcb/tests/test_board_spec.py`:

```python
import unittest
from collections import Counter
from pcb import board_spec as B, tools
from pcb.sexpr import parse, find

def lib_symbols(lib):
    return {c[1]: c for c in find(parse((tools.SYMBOL_DIR / f'{lib}.kicad_sym').read_text())[0], 'symbol')}

class SpecTests(unittest.TestCase):
    def test_pico_pin_map_matches_spec_section_4(self):
        want = {'GP0': 'ENC_LA_3V3', 'GP1': 'ENC_LB_3V3', 'GP2': 'ENC_RA_3V3', 'GP3': 'ENC_RB_3V3', 'GP4': 'SDA', 'GP5': 'SCL',
                'GP6': 'L_IN1', 'GP7': 'L_IN2', 'GP8': 'R_IN1', 'GP9': 'R_IN2', 'GP10': 'XSHUT1', 'GP11': 'XSHUT2',
                'GP12': 'XSHUT3', 'GP13': 'XSHUT4', 'GP14': 'BUMP_L', 'GP15': 'BUMP_R', 'GP16': 'MOTOR_ON_N', 'GP17': 'ARM',
                'GP18': 'DRV_SLEEP', 'GP19': 'DRV_FAULT', 'GP20': 'SPARE20', 'GP21': 'SPARE21', 'GP22': 'SPARE22',
                'GP26': 'VSENSE', 'GP27': 'L_CS', 'GP28': 'R_CS', '3V3_OUT': '3V3'}
        for gp, net in want.items():
            self.assertIn(B.pico(gp), B.NETS[net], f'{gp} must be on {net}')
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

if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 3: Run it to verify it fails**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_board_spec -v`
Expected: ERROR `No module named 'pcb.board_spec'`.

- [ ] **Step 4: Record the vendor pinouts and library names** (research, results go into Step 5's data):
  1. Fetch https://www.pololu.com/product/4035 (DRV8874 carrier). Save the "Pinout" text to `pcb/reference/pololu-4035.txt`. From it record the two header rows (pin count of each row, the pin order along each row, row spacing in mm and the carrier outline) and which mode-pin level selects PWM/IN-IN control, and the current-sense output (V/A).
  2. Fetch https://www.pololu.com/product/2851 (D24V50F5) and save its pin order to `pcb/reference/pololu-2851.txt`.
  3. Relay: `grep -n 'G5LE' $SYMBOL_DIR/Relay.kicad_sym` and read the pin numbers and names of the symbol you choose. Its footprint must be `Relay_THT:Relay_SPDT_Omron-G5LE-1`. Map coil+, coil−, COM, NO and NC to numbers.
  4. Fuse holder: `ls $FOOTPRINT_DIR/Fuse.pretty | grep -i -E 'blade|ato|3557'`. If no ATO/ATC PCB blade holder exists, create `pcb/r1.pretty/Fuseholder_Blade_ATO_Keystone_3557-2.kicad_mod` from the Keystone 3557-2 datasheet (4 pins, two per blade terminal, pad numbers 1,1,2,2).
  5. If the carrier has more than one GND pin, record the first as `DRV_PINS['GND']` and the rest in `DRV_EXTRA_GND`.
  6. Ask the user to measure the existing 5 V converter (P03): outline, terminal positions and mounting-hole positions/diameter. If they provide them, add one `Mechanical:MountingHole` part per hole (`fp='MountingHole:MountingHole_3.2mm_M3'`, no pins, so nothing goes into `NETS`) placed relative to `U5A`, and size the regulator keep-out from the outline. If they cannot, keep `U5A`/`U5B` screw terminals only and note in `ORDERING.md` that the module is held with double-sided foam tape in its keep-out.

- [ ] **Step 5: Write `pcb/board_spec.py`** with this structure and the recorded values:

```python
"""R1 power & I/O board — single source of truth (spec 2026-10-04-r1-power-io-board-design.md, sections 3-7).
Pure Python (no numpy): imported by the CAD venv and by KiCad's bundled Python."""
from pathlib import Path
from pcb.sexpr import parse, find
from pcb import tools

# Board frame: u = robot x - BOARD_X0 (toward the robot front), v = BOARD_Y0 - robot y (toward robot right).
BOARD_X0, BOARD_Y0, BOARD_W, BOARD_H = 14.0, 85.0, 84.0, 170.0
Z_PCB_BOTTOM, PCB_T = 39.0, 1.6
MAX_PART_H = 78.0 - (Z_PCB_BOTTOM + PCB_T) - 0.4          # deck underside z 78, 0.4 mm margin
CUSTOM_FP_DIR = Path(__file__).parent / 'r1.pretty'
def robot_to_board(x, y): return (x - BOARD_X0, BOARD_Y0 - y)
def board_to_robot(u, v): return (u + BOARD_X0, BOARD_Y0 - v)

# --- Vendor pinouts (Task 2 Step 4; sources in pcb/reference/) ---
DRV_ROWS = {'L': 0, 'P': 0}        # pins in the carrier's logic row and power row (from pololu-4035.txt)
DRV_PINS = {}                      # name -> (row 'L'|'P', pin number 1..n) for VIN GND OUT1 OUT2 IN1 IN2 PMODE SLEEP FAULT CS
DRV_PMODE_NET = '3V3'              # '3V3' or 'GND': whichever level selects PWM (IN/IN) mode per pololu-4035.txt
DRV_EXTRA_GND = []                 # further carrier GND pins beyond DRV_PINS['GND']: (row, pin), tied to GND
DRV_UNUSED = []                    # remaining carrier pins left unconnected (e.g. VREF if the carrier sets it): (row, pin)
REG_B_PINS = {}                    # D24V50F5 name -> pin number (VIN, GND, VOUT, EN, PG ...) from pololu-2851.txt
RELAY_SYM = 'Relay:G5LE-1'         # the Relay library symbol chosen in Step 4.3
RELAY_PINS = {}                    # COIL_P COIL_N COM NO NC -> symbol pin numbers (Step 4.3)
FUSE_FP = 'R1:Fuseholder_Blade_ATO_Keystone_3557-2'   # or the standard library holder found in Step 4.4

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

FP = dict(VH2='Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical', XH2='Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical',
          XH4='Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical', XH5='Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical',
          HDR6='Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical',
          R='Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal', C='Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm',
          CP='Capacitor_THT:CP_Radial_D10.0mm_P5.00mm', Q='Package_TO_SOT_THT:TO-92_Inline',
          D='Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal', LED='LED_THT:LED_D3.0mm',
          TP='TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm', HOLE='MountingHole:MountingHole_3.2mm_M3_Pad_Via',
          TERM2='TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal')
def sock(n): return f'Connector_PinSocket_2.54mm:PinSocket_1x{n:02d}_P2.54mm_Vertical'
def conn(n): return f'Connector_Generic:Conn_01x{n:02d}'

def P(sym, fp, value, mpn='', h=10.0): return dict(sym=sym, fp=fp, value=value, mpn=mpn, h=h)
PARTS = {
    'J1': P(conn(2), FP['VH2'], 'Battery in', 'JST B2P-VH(LF)(SN)'), 'J2': P(conn(2), FP['VH2'], 'Master rocker', 'JST B2P-VH(LF)(SN)'),
    'J3': P(conn(2), FP['VH2'], 'Jetson 5V', 'JST B2P-VH(LF)(SN)'),
    'J4': P(conn(6), FP['HDR6'], 'Left motor'), 'J5': P(conn(6), FP['HDR6'], 'Right motor'),
    **{f'J{6 + i}': P(conn(5), FP['XH5'], f'Cliff {i + 1}', 'JST B5B-XH-A(LF)(SN)') for i in range(4)},
    'J10': P(conn(2), FP['XH2'], 'Bumper L', 'JST B2B-XH-A(LF)(SN)'), 'J11': P(conn(2), FP['XH2'], 'Bumper R', 'JST B2B-XH-A(LF)(SN)'),
    'J12': P(conn(2), FP['XH2'], 'Arm button', 'JST B2B-XH-A(LF)(SN)'), 'J13': P(conn(2), FP['XH2'], 'Motor toggle loop', 'JST B2B-XH-A(LF)(SN)'),
    'J14': P(conn(4), FP['XH4'], 'Spare I2C', 'JST B4B-XH-A(LF)(SN)'), 'J15': P(conn(6), FP['HDR6'], 'Spare GPIO'),
    'U1A': P(conn(20), sock(20), 'Pico left row', h=12), 'U1B': P(conn(20), sock(20), 'Pico right row', h=12),
    'U2': P(conn(8), sock(8), 'GY-521 MPU-6050', h=14),
    'U3L': P(conn(DRV_ROWS['L'] or 1), sock(DRV_ROWS['L'] or 1), 'DRV8874 left, logic row', 'Pololu 4035', 14),
    'U3P': P(conn(DRV_ROWS['P'] or 1), sock(DRV_ROWS['P'] or 1), 'DRV8874 left, power row', 'Pololu 4035', 14),
    'U4L': P(conn(DRV_ROWS['L'] or 1), sock(DRV_ROWS['L'] or 1), 'DRV8874 right, logic row', 'Pololu 4035', 14),
    'U4P': P(conn(DRV_ROWS['P'] or 1), sock(DRV_ROWS['P'] or 1), 'DRV8874 right, power row', 'Pololu 4035', 14),
    'U5A': P(conn(2), FP['TERM2'], 'Regulator in (P03 module)', h=12), 'U5B': P(conn(2), FP['TERM2'], 'Regulator out (P03 module)', h=12),
    'U5C': P(conn(len(REG_B_PINS) or 1), sock(len(REG_B_PINS) or 1), 'D24V50F5 (P04) socket', 'Pololu 2851', 14),
    'F1': P('Device:Fuse', FUSE_FP, '7.5 A ATO', 'Keystone 3557-2', 16), 'F2': P('Device:Fuse', FUSE_FP, '5 A ATO', 'Keystone 3557-2', 16),
    'K1': P(RELAY_SYM, 'Relay_THT:Relay_SPDT_Omron-G5LE-1', 'G5LE-1 DC5', 'Omron G5LE-1 DC5', 19),
    'D1': P('Device:D_Schottky', FP['D'], '1N5819', '1N5819'),
    'C1': P('Device:C_Polarized', FP['CP'], '470uF 25V', 'Panasonic EEU-FR1E471', 20),
    'C2': P('Device:C', FP['C'], '100nF', 'X7R 50 V'),
    **{f'Q{i}': P('Device:Q_NMOS_SGD', FP['Q'], '2N7000', 'onsemi 2N7000', 6) for i in range(1, 6)},
    **{f'LED{i}': P('Device:LED', FP['LED'], c, h=6) for i, c in ((1, 'green'), (2, 'green'), (3, 'yellow'))},
    **{f'TP{i}': P('Connector:TestPoint', FP['TP'], n) for i, n in enumerate(('VBAT_SW', 'VMOT', '5V', '3V3', 'GND'), 1)},
    **{f'H{i}': P('Mechanical:MountingHole_Pad', FP['HOLE'], 'M3', h=0) for i in range(1, 5)},
}
RES = {  # ref: (value, net a, net b)
    'R1': ('100k', 'ENC_LA_5V', 'GND'), 'R2': ('10k', 'ENC_LA_3V3', '3V3'), 'R3': ('100k', 'ENC_LB_5V', 'GND'), 'R4': ('10k', 'ENC_LB_3V3', '3V3'),
    'R5': ('100k', 'ENC_RA_5V', 'GND'), 'R6': ('10k', 'ENC_RA_3V3', '3V3'), 'R7': ('100k', 'ENC_RB_5V', 'GND'), 'R8': ('10k', 'ENC_RB_3V3', '3V3'),
    'R9': ('100k', 'COIL_P', 'GND'), 'R10': ('10k', 'MOTOR_ON_N', '3V3'),
    'R11': ('100k', 'VBAT_SW', 'VSENSE'), 'R12': ('33k', 'VSENSE', 'GND'),
    'R13': ('10k', 'L_IN1', 'GND'), 'R14': ('10k', 'L_IN2', 'GND'), 'R15': ('10k', 'R_IN1', 'GND'), 'R16': ('10k', 'R_IN2', 'GND'),
    'R17': ('100k', 'DRV_SLEEP', 'GND'), 'R18': ('10k', 'DRV_FAULT', '3V3'),
    'R19': ('4.7k', 'SDA', '3V3'), 'R20': ('4.7k', 'SCL', '3V3'),
    'R21': ('3.3k', 'VBAT_SW', 'LED1_A'), 'R22': ('1k', '5V', 'LED2_A'), 'R23': ('3.3k', 'VMOT', 'LED3_A'),
}
PARTS.update({r: P('Device:R', FP['R'], v, 'metal film 0.25 W 1 %', 3) for r, (v, a, b) in RES.items()})

def _nets():
    n = {}
    def add(net, *pins): n.setdefault(net, []).extend(pins)
    for r, (v, a, b) in RES.items(): add(a, (r, '1')); add(b, (r, '2'))
    add('BAT+', ('J1', '1'), ('F1', '1')); add('F1_OUT', ('F1', '2'), ('J2', '1'))
    add('VBAT_SW', ('J2', '2'), ('F2', '1'), ('U5A', '1'), ('TP1', '1'))
    add('F2_OUT', ('F2', '2'), ('K1', RELAY_PINS['COM']))
    add('VMOT', ('K1', RELAY_PINS['NO']), ('C1', '1'), drv('L', 'VIN'), drv('R', 'VIN'), ('TP2', '1'))
    add('5V', ('U5B', '1'), ('J3', '1'), ('J4', '4'), ('J5', '4'), ('J13', '1'), ('J15', '5'), ('TP3', '1'))
    add('3V3', pico('3V3_OUT'), imu('VCC'), *[(f'J{6 + i}', '1') for i in range(4)], ('J14', '1'), ('J15', '4'), ('TP4', '1'))
    if DRV_PMODE_NET == '3V3': add('3V3', drv('L', 'PMODE'), drv('R', 'PMODE'))
    gnd = [('J1', '2'), ('J3', '2'), ('J4', '3'), ('J5', '3'), ('J10', '2'), ('J11', '2'), ('J12', '2'), ('J14', '2'), ('J15', '6'),
           ('U5A', '2'), ('U5B', '2'), ('C1', '2'), ('C2', '2'), ('K1', RELAY_PINS['COIL_N']), ('TP5', '1'),
           *[(f'J{6 + i}', '2') for i in range(4)], *[pico(g) for g in ('GND1', 'GND2', 'GND3', 'GND4', 'GND5', 'GND6', 'AGND', 'GND7')],
           imu('GND'), imu('AD0'), drv('L', 'GND'), drv('R', 'GND'), *[(f'H{i}', '1') for i in range(1, 5)], ('D1', '2'),
           *[(f'U{u}{row}', str(pin)) for u in (3, 4) for (row, pin) in DRV_EXTRA_GND],
           *[(f'Q{i}', '1') for i in range(1, 6)], *[(f'LED{i}', '1') for i in range(1, 4)]]
    if DRV_PMODE_NET == 'GND': gnd += [drv('L', 'PMODE'), drv('R', 'PMODE')]
    add('GND', *gnd)
    if REG_B_PINS:   # D24V50F5 socket in parallel with the P03 terminals (fit only one regulator)
        add('VBAT_SW', ('U5C', str(REG_B_PINS['VIN']))); add('5V', ('U5C', str(REG_B_PINS['VOUT'])))
        add('GND', *[('U5C', str(p)) for k, p in REG_B_PINS.items() if k.startswith('GND')])
    add('COIL_P', ('J13', '2'), ('K1', RELAY_PINS['COIL_P']), ('D1', '1'), ('Q5', '2'))   # D1 cathode on coil+, anode on GND
    for side, j in (('L', 'J4'), ('R', 'J5')):
        add(f'{side}_M1', (j, '1'), drv(side, 'OUT1')); add(f'{side}_M2', (j, '2'), drv(side, 'OUT2'))
        add(f'{side}_CS', drv(side, 'CS')); add(f'{side}_IN1', drv(side, 'IN1')); add(f'{side}_IN2', drv(side, 'IN2'))
    add('L_CS', pico('GP27')); add('R_CS', pico('GP28'))
    add('L_IN1', pico('GP6')); add('L_IN2', pico('GP7')); add('R_IN1', pico('GP8')); add('R_IN2', pico('GP9'))
    add('DRV_SLEEP', pico('GP18'), drv('L', 'SLEEP'), drv('R', 'SLEEP')); add('DRV_FAULT', pico('GP19'), drv('L', 'FAULT'), drv('R', 'FAULT'))
    for i, (sig, j, pin, gp) in enumerate((('LA', 'J4', '5', 'GP0'), ('LB', 'J4', '6', 'GP1'), ('RA', 'J5', '5', 'GP2'), ('RB', 'J5', '6', 'GP3')), 1):
        add(f'ENC_{sig}_5V', (j, pin), (f'Q{i}', '2')); add(f'ENC_{sig}_3V3', (f'Q{i}', '3'), pico(gp))
    add('MOTOR_ON_N', ('Q5', '3'), pico('GP16'))
    add('SDA', pico('GP4'), imu('SDA'), *[(f'J{6 + i}', '3') for i in range(4)], ('J14', '3'))
    add('SCL', pico('GP5'), imu('SCL'), *[(f'J{6 + i}', '4') for i in range(4)], ('J14', '4'))
    for i, gp in enumerate(('GP10', 'GP11', 'GP12', 'GP13')): add(f'XSHUT{i + 1}', (f'J{6 + i}', '5'), pico(gp))
    add('BUMP_L', ('J10', '1'), pico('GP14')); add('BUMP_R', ('J11', '1'), pico('GP15')); add('ARM', ('J12', '1'), pico('GP17'))
    add('VSENSE', pico('GP26'), ('C2', '1'))
    for i, gp in ((1, 'GP20'), (2, 'GP21'), (3, 'GP22')): add(f'SPARE2{i - 1}', ('J15', str(i)), pico(gp))
    for i in range(1, 4): add(f'LED{i}_A', (f'LED{i}', '2'))
    return n
```

Then, below `_nets`, add:

```python
NETS = _nets()
NC = ([pico(n) for n in ('RUN', 'ADC_VREF', '3V3_EN', 'VSYS', 'VBUS')] + [imu(n) for n in ('XDA', 'XCL', 'INT')]
      + [('K1', RELAY_PINS['NC'])] + [(f'U{u}{row}', str(pin)) for u in (3, 4) for (row, pin) in DRV_UNUSED]
      + [('U5C', str(pin)) for name, pin in REG_B_PINS.items() if name not in ('VIN', 'VOUT') and not name.startswith('GND')])
```

Then add:

```python
NETCLASS_POWER = ['BAT+', 'F1_OUT', 'VBAT_SW', 'F2_OUT', 'VMOT', '5V', 'GND', 'L_M1', 'L_M2', 'R_M1', 'R_M2']
CURRENT_A = {'BAT+': 7.5, 'F1_OUT': 7.5, 'VBAT_SW': 7.5, 'F2_OUT': 5.0, 'VMOT': 5.0, '5V': 4.5, 'GND': 7.5,
             'L_M1': 2.1, 'L_M2': 2.1, 'R_M1': 2.1, 'R_M2': 2.1}
REMOVED_CAD_IDS = ['regulator', 'driver', 'driver_cap', 'relay_carrier', 'relay', 'fuse_1', 'fuse_2', 'battery_bus', 'ground_bus',
                   'controller_carrier', 'pico', 'pico_usb', 'pico_chip', 'imu', 'J02', 'J03', 'J04', 'J07', 'J08', 'J09', 'J18_micro']
def symbol_pin_numbers(lib_id):
    lib, name = lib_id.split(':')
    syms = {c[1]: c for c in find(parse((tools.SYMBOL_DIR / f'{lib}.kicad_sym').read_text())[0], 'symbol')}
    s = syms[name]
    if find(s, 'extends'): s = syms[find(s, 'extends')[0][1]]
    return {str(find(p, 'number')[0][1]) for sub in find(s, 'symbol') for p in find(sub, 'pin')}
# Board-frame placement (u, v, rotation deg). Zones: power end v 55-125 aft (u < 45), logic end front (u > 45).
PLACEMENT = {
    'J1': (5, 86, 90), 'J2': (5, 72, 90), 'J3': (5, 100, 90), 'F1': (20, 74, 0), 'F2': (20, 98, 0), 'K1': (36, 84, 0), 'D1': (36, 100, 0),
    'C1': (36, 66, 0), 'J4': (4, 22, 90), 'J5': (4, 148, 90), 'U3L': (16, 18, 0), 'U3P': (16, 36, 0), 'U4L': (16, 134, 0), 'U4P': (16, 152, 0),
    'U5A': (30, 118, 0), 'U5B': (44, 118, 0), 'U5C': (38, 132, 90), 'U1A': (52, 40, 90), 'U1B': (70, 40, 90), 'U2': (80, 110, 90),
    'J6': (80, 8, 270), 'J7': (80, 162, 270), 'J8': (6, 8, 0), 'J9': (6, 162, 0), 'J10': (80, 24, 270), 'J11': (80, 146, 270),
    'J12': (60, 100, 0), 'J13': (48, 100, 0), 'J14': (80, 128, 270), 'J15': (60, 150, 90),
    **{f'Q{i}': (28 + 5 * i, 50, 0) for i in range(1, 6)}, 'C2': (60, 112, 0),
    **{f'LED{i}': (28 + 6 * i, 140, 0) for i in range(1, 4)}, **{f'TP{i}': (24 + 6 * i, 160, 0) for i in range(1, 6)},
    'H1': (4, 4, 0), 'H2': (80, 4, 0), 'H3': (4, 166, 0), 'H4': (80, 166, 0),
    **{r: (24 + 8 * (k % 6), 58 + 6 * (k // 6), 90) for k, r in enumerate(RES)},
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_board_spec -v`
Expected: 7 tests OK. Fix data (not tests) until they pass; every pin of every symbol is netted or listed in `NC`.

- [ ] **Step 7: Archive** (`<N>`=2).

---

### Task 3: Board outline from the robot CAD

**Files:**
- Create: `$R1/pcb/extract_outline.py`, `$R1/pcb/board_outline.json` (generated), `$R1/pcb/tests/test_outline.py`

**Interfaces:**
- Consumes: `$R1/cad/component-manifest.json` (list of parts with `id`, `bounds` [xmin,ymin,zmin,xmax,ymax,zmax] mm), `board_spec.REMOVED_CAD_IDS`, `robot_to_board`.
- Produces: `board_outline.json` = `{"outline_uv": [[u, v], ...] (closed polygon, mm), "holes_uv": [[u, v] x4], "keepouts_robot": [{"id", "bounds"}], "max_part_h": float}`.

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/test_outline.py`:

```python
import json, unittest
from pathlib import Path
from pcb import board_spec as B
OUT = Path(B.__file__).parent / 'board_outline.json'
def inside(poly, p):
    x, y = p; c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1: c = not c
    return c
def area(poly): return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]))) / 2

class OutlineTests(unittest.TestCase):
    def setUp(self): self.o = json.loads(OUT.read_text())
    def test_large_enough(self): self.assertGreater(area(self.o['outline_uv']), 9000)
    def test_holes_inside_with_margin(self):
        for u, v in self.o['holes_uv']:
            for du, dv in ((4, 0), (-4, 0), (0, 4), (0, -4)): self.assertTrue(inside(self.o['outline_uv'], (u + du, v + dv)))
    def test_keepouts_outside_outline(self):
        for k in self.o['keepouts_robot']:
            x0, y0, z0, x1, y1, z1 = k['bounds']
            for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1), ((x0 + x1) / 2, (y0 + y1) / 2)):
                self.assertFalse(inside(self.o['outline_uv'], B.robot_to_board(x, y)), k['id'])
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_outline -v`
Expected: ERROR `FileNotFoundError ... board_outline.json`.

- [ ] **Step 3: Write `pcb/extract_outline.py`**

```python
"""Board outline = envelope rectangle minus every remaining CAD part that reaches into the board slab (inflated 1.5 mm)."""
import json, sys
from pathlib import Path
import cadquery as cq
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B
ROOT = Path(__file__).resolve().parents[1]
parts = json.loads((ROOT / 'cad/component-manifest.json').read_text())
parts = parts if isinstance(parts, list) else parts['parts']
Z0, Z1 = B.Z_PCB_BOTTOM - 1, 78.0                                  # board + components slab (robot z)
ENV = (B.BOARD_X0, B.BOARD_Y0 - B.BOARD_H, B.BOARD_X0 + B.BOARD_W, B.BOARD_Y0)
skip = set(B.REMOVED_CAD_IDS) | {'P_base', 'P_deck'}
face = cq.Workplane('XY').rect(ENV[2] - ENV[0], ENV[3] - ENV[1], centered=False).translate((ENV[0], ENV[1], 0)).extrude(1)
keep = []
for p in parts:
    x0, y0, z0, x1, y1, z1 = p['bounds']
    if p['id'] in skip or p['id'].startswith('W') or z1 < Z0 or z0 > Z1: continue
    if x1 < ENV[0] or x0 > ENV[2] or y1 < ENV[1] or y0 > ENV[3]: continue
    keep.append(dict(id=p['id'], bounds=p['bounds']))
    face = face.cut(cq.Workplane('XY').box(x1 - x0 + 3, y1 - y0 + 3, 10).translate(((x0 + x1) / 2, (y0 + y1) / 2, 0)))
wire = face.faces('<Z').val().outerWire()
pts = [B.robot_to_board(v.X, v.Y) for v in wire.Vertices()]
holes = [(5, 5), (B.BOARD_W - 5, 5), (5, B.BOARD_H - 5), (B.BOARD_W - 5, B.BOARD_H - 5)]
out = dict(outline_uv=[[round(u, 2), round(v, 2)] for u, v in pts], holes_uv=holes, keepouts_robot=keep, max_part_h=B.MAX_PART_H)
(Path(__file__).parent / 'board_outline.json').write_text(json.dumps(out, indent=1))
print(len(pts), 'outline points;', len(keep), 'keep-outs:', [k['id'] for k in keep])
```

The vertex order of `outerWire()` follows the face boundary. If the test reports a hole outside the outline, move that hole inward along the diagonal in 2 mm steps inside the script (loop until `test_holes_inside_with_margin`'s rule holds), and write the result.

- [ ] **Step 4: Run it, then the test**

Run: `cd $R1 && $CADPY pcb/extract_outline.py && $CADPY -m unittest pcb.tests.test_outline -v`
Expected: the keep-out list printed (motor mounts, camera-tower feet, cliff mounts, if they reach the envelope); 3 tests OK.

- [ ] **Step 5: Archive** (`<N>`=3).

---

### Task 4: Schematic (net-label style) with KiCad ERC

**Files:**
- Create: `$R1/pcb/make_schematic.py`, generated `$R1/pcb/r1_power_io.kicad_sch`, `$R1/pcb/r1_power_io.kicad_pro`, `$R1/pcb/r1_power_io.net`, `$R1/pcb/tests/test_schematic.py`

**Interfaces:**
- Consumes: `board_spec.PARTS/NETS/NC`, `sexpr`, `tools.SYMBOL_DIR`, `tools.KICAD_CLI`.
- Produces: `make_schematic.main()` writes the files above. `pcb.make_schematic.read_netlist(path) -> dict[str, set[tuple[str, str]]]` parses a KiCad S-expression netlist.

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/test_schematic.py`:

```python
import json, subprocess, unittest
from pathlib import Path
from pcb import board_spec as B, tools
from pcb.make_schematic import read_netlist
D = Path(B.__file__).parent

class SchematicTests(unittest.TestCase):
    def test_erc_has_no_errors(self):
        subprocess.run([tools.KICAD_CLI, 'sch', 'erc', '--format', 'json', '--severity-error', '-o', str(D / 'erc.json'),
                        str(D / 'r1_power_io.kicad_sch')], check=True, capture_output=True)
        rep = json.loads((D / 'erc.json').read_text())
        errs = [v for s in rep['sheets'] for v in s['violations']]
        self.assertEqual(errs, [], json.dumps(errs[:5], indent=1))
    def test_netlist_equals_board_spec(self):
        subprocess.run([tools.KICAD_CLI, 'sch', 'export', 'netlist', '--format', 'kicadsexpr', '-o', str(D / 'r1_power_io.net'),
                        str(D / 'r1_power_io.kicad_sch')], check=True, capture_output=True)
        got = read_netlist(D / 'r1_power_io.net')
        want = {n: set(p) for n, p in B.NETS.items()}
        self.assertEqual(got, want)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_schematic -v`
Expected: ERROR `No module named 'pcb.make_schematic'`.

- [ ] **Step 3: Write `pcb/make_schematic.py`**

```python
"""Net-label schematic from board_spec: every symbol on a grid, a local label on every netted pin, a no-connect flag on every NC pin."""
import copy, json, sys, uuid
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, tools
from pcb.sexpr import Q, parse, dump, find
D = Path(__file__).parent
U = lambda: Q(str(uuid.uuid4()))
_libs = {}
def lib_symbol(lib_id):
    lib, name = lib_id.split(':')
    if lib not in _libs:
        _libs[lib] = {c[1]: c for c in find(parse((tools.SYMBOL_DIR / f'{lib}.kicad_sym').read_text())[0], 'symbol')}
    s = copy.deepcopy(_libs[lib][name])
    ext = find(s, 'extends')
    if ext:   # derived symbol: parent's drawing and pins, child's properties
        parent = copy.deepcopy(_libs[lib][ext[0][1]])
        body = [c for c in parent[2:] if not (isinstance(c, list) and c[0] == 'property')]
        for c in body:
            if isinstance(c, list) and c[0] == 'symbol': c[1] = Q(c[1].replace(ext[0][1], name, 1))
        s = ['symbol', Q(name)] + [c for c in s[2:] if not (isinstance(c, list) and c[0] == 'extends')] + body
    s[1] = Q(lib_id)
    return s
def pin_points(sym):   # pin number -> (x, y) electrical end in symbol coordinates (y up)
    return {str(find(p, 'number')[0][1]): (float(find(p, 'at')[0][1]), float(find(p, 'at')[0][2]))
            for sub in find(sym, 'symbol') for p in find(sub, 'pin')}
def prop(k, v, x, y, hide=False):
    eff = ['effects', ['font', ['size', '1.27', '1.27']]] + ([['hide', 'yes']] if hide else [])
    return ['property', Q(k), Q(v), ['at', f'{x:.2f}', f'{y:.2f}', '0'], eff]
def read_netlist(path):
    tree = parse(Path(path).read_text())[0]; out = {}
    for net in find(find(tree, 'nets')[0], 'net'):
        out[str(find(net, 'name')[0][1])] = {(str(find(n, 'ref')[0][1]), str(find(n, 'pin')[0][1])) for n in find(net, 'node')}
    return out
def main():
    root = U(); pin_net = {p: n for n, pins in B.NETS.items() for p in pins}
    libsyms, items, used = [], [], set()
    for k, (ref, part) in enumerate(sorted(B.PARTS.items())):
        sym = lib_symbol(part['sym'])
        if part['sym'] not in used: libsyms.append(sym); used.add(part['sym'])
        X, Y = 25.4 + 50.8 * (k % 8), 25.4 + 63.5 * (k // 8)           # 1.27-grid aligned
        pins = pin_points(sym)
        inst = ['symbol', ['lib_id', Q(part['sym'])], ['at', f'{X:.2f}', f'{Y:.2f}', '0'], ['unit', '1'],
                ['in_bom', 'yes'], ['on_board', 'yes'], ['dnp', 'no'], ['uuid', U()],
                prop('Reference', ref, X, Y - 3), prop('Value', part['value'], X, Y + 3), prop('Footprint', part['fp'], X, Y, True)]
        inst += [['pin', Q(n), ['uuid', U()]] for n in pins]
        inst += [['instances', ['project', Q('r1_power_io'), ['path', Q('/' + root), ['reference', Q(ref)], ['unit', '1']]]]]
        items.append(inst)
        for n, (px, py) in pins.items():
            x, y = X + px, Y - py
            if (ref, n) in pin_net:
                items.append(['label', Q(pin_net[(ref, n)]), ['at', f'{x:.2f}', f'{y:.2f}', '0'],
                              ['effects', ['font', ['size', '1.0', '1.0']], ['justify', 'left', 'bottom']], ['uuid', U()]])
            else:
                items.append(['no_connect', ['at', f'{x:.2f}', f'{y:.2f}'], ['uuid', U()]])
    sch = ['kicad_sch', ['version', '20231120'], ['generator', Q('r1_make_schematic')], ['uuid', root], ['paper', Q('A0')],
           ['lib_symbols', *libsyms], *items, ['sheet_instances', ['path', Q('/'), ['page', Q('1')]]]]
    (D / 'r1_power_io.kicad_sch').write_text(dump(sch))
    pro = {'meta': {'filename': 'r1_power_io.kicad_pro', 'version': 1}, 'net_settings': {
        'classes': [{'name': 'Default', 'track_width': 0.3, 'clearance': 0.25, 'via_diameter': 0.8, 'via_drill': 0.4},
                    {'name': 'Power', 'track_width': 3.0, 'clearance': 0.4, 'via_diameter': 1.2, 'via_drill': 0.6}],
        'netclass_patterns': [{'netclass': 'Power', 'pattern': n} for n in B.NETCLASS_POWER]}}
    (D / 'r1_power_io.kicad_pro').write_text(json.dumps(pro, indent=2))
if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Generate and run the tests**

Run: `cd $R1 && $CADPY pcb/make_schematic.py && $CADPY -m unittest pcb.tests.test_schematic -v`
Expected: 2 tests OK. If ERC reports a pin-type rule that is not a wiring error (for example "power input pin not driven" on a library symbol), set that rule to `warning` in `r1_power_io.kicad_pro` under `erc.rule_severities` in `make_schematic.py`, and note it in the archive message. Never silence `pin_not_connected` or `label_dangling`.

- [ ] **Step 5: Archive** (`<N>`=4).

---

### Task 5: Placed PCB

**Files:**
- Create: `$R1/pcb/build_pcb.py`, generated `$R1/pcb/r1_power_io.kicad_pcb`, `$R1/pcb/tests/ktest_pcb.py`

**Interfaces:**
- Consumes: `board_spec` (`PARTS`, `NETS`, `PLACEMENT`), `board_outline.json`, `tools.FOOTPRINT_DIR`, `CUSTOM_FP_DIR`.
- Produces: `r1_power_io.kicad_pcb`, with all footprints placed and every pad carrying its `NETS` net, the Edge.Cuts outline, M3 holes, and a GND zone on B.Cu (unfilled).

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/ktest_pcb.py` (runs with `$KPY`):

```python
import json, unittest
from pathlib import Path
import pcbnew
from pcb import board_spec as B
D = Path(B.__file__).parent

class PcbTests(unittest.TestCase):
    def setUp(self): self.b = pcbnew.LoadBoard(str(D / 'r1_power_io.kicad_pcb'))
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
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_pcb -v`
Expected: ERROR (board file missing).

- [ ] **Step 3: Write `pcb/build_pcb.py`**

```python
"""Place footprints from board_spec, assign nets, draw the outline, holes and the GND zone. Run with KiCad's Python."""
import json, sys
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, tools
D = Path(__file__).parent; MM = pcbnew.FromMM
V = lambda u, v: pcbnew.VECTOR2I(MM(u), MM(v))
board = pcbnew.BOARD()
board.SetCopperLayerCount(2)
outline = json.loads((D / 'board_outline.json').read_text())
pts = outline['outline_uv']
for (u1, v1), (u2, v2) in zip(pts, pts[1:] + pts[:1]):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
    s.SetStart(V(u1, v1)); s.SetEnd(V(u2, v2)); s.SetWidth(MM(0.1)); board.Add(s)
nets = {}
for name in B.NETS:
    ni = pcbnew.NETINFO_ITEM(board, name); board.Add(ni); nets[name] = ni
pin_net = {p: n for n, pins in B.NETS.items() for p in pins}
for ref, part in B.PARTS.items():
    lib, name = part['fp'].split(':')
    path = B.CUSTOM_FP_DIR if lib == 'R1' else tools.FOOTPRINT_DIR / f'{lib}.pretty'
    fp = pcbnew.FootprintLoad(str(path), name)
    fp.SetReference(ref); fp.SetValue(part['value'])
    u, v, rot = (outline['holes_uv'][int(ref[1]) - 1] + [0]) if ref.startswith('H') else B.PLACEMENT[ref]
    fp.SetPosition(V(u, v)); fp.SetOrientationDegrees(rot)
    for pad in fp.Pads():
        n = pin_net.get((ref, pad.GetNumber()))
        if n: pad.SetNet(nets[n])
    board.Add(fp)
zone = pcbnew.ZONE(board); zone.SetLayer(pcbnew.B_Cu); zone.SetNet(nets['GND'])
o = zone.Outline(); o.NewOutline()
for u, v in pts: o.Append(MM(u), MM(v))
zone.SetLocalClearance(MM(0.4)); zone.SetMinThickness(MM(0.3)); board.Add(zone)
board.Save(str(D / 'r1_power_io.kicad_pcb'))
print('placed', len(B.PARTS), 'footprints')
```

If KiCad 9 renamed an API call (`FootprintLoad`, `SetOrientationDegrees`, `SetLocalClearance`), check the replacement with `$KPY -c "import pcbnew;help(pcbnew.FOOTPRINT)"` and use it.

- [ ] **Step 4: Generate and run the test**

Run: `cd $R1 && $KPY pcb/build_pcb.py && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_pcb -v`
Expected: 3 tests OK. If `test_courtyards_inside_outline` fails, move the reported parts in `board_spec.PLACEMENT` (data, not tests) and regenerate.

- [ ] **Step 5: Check courtyard overlaps with DRC before routing**

Run: `$KCLI pcb drc --format json --severity-error -o $R1/pcb/drc_placed.json $R1/pcb/r1_power_io.kicad_pcb && $CADPY -c "import json;r=json.load(open('$R1/pcb/drc_placed.json'));print([v['type'] for v in r['violations'] if v['type']=='courtyards_overlap'])"`
Expected: `[]`. Otherwise adjust `PLACEMENT` and repeat Steps 4–5.

- [ ] **Step 6: Archive** (`<N>`=5).

---

### Task 6: Routing and DRC

**Files:**
- Create: `$R1/pcb/route.py`, `$R1/pcb/tests/ktest_route.py`

**Interfaces:**
- Consumes: `r1_power_io.kicad_pcb` (Task 5), the `.kicad_pro` net classes (Task 4: Power 3.0 mm/0.4 mm, Default 0.3 mm/0.25 mm), `tools.JAVA/FREEROUTING/KICAD_CLI`.
- Produces: routed `r1_power_io.kicad_pcb` with filled zones; `drc.json`.

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/ktest_route.py`:

```python
import json, subprocess, unittest
from pathlib import Path
from pcb import board_spec as B, tools
D = Path(B.__file__).parent

class RouteTests(unittest.TestCase):
    def test_drc_clean_and_fully_routed(self):
        subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--severity-error',
                        '-o', str(D / 'drc.json'), str(D / 'r1_power_io.kicad_pcb')], check=True, capture_output=True)
        rep = json.loads((D / 'drc.json').read_text())
        self.assertEqual(rep['violations'], []); self.assertEqual(rep['unconnected_items'], [])
        # Schematic parity is covered by Task 4 (exported netlist == board_spec) and Task 5 (pad nets == board_spec).
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_route -v`
Expected: FAIL: `unconnected_items` is not empty (nothing routed yet).

- [ ] **Step 3: Write `pcb/route.py`**

```python
"""Export DSN, autoroute with Freerouting (headless), import the session, fill zones, save."""
import subprocess, sys
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import tools
D = Path(__file__).parent; PCB = D / 'r1_power_io.kicad_pcb'
board = pcbnew.LoadBoard(str(PCB))
pcbnew.ExportSpecctraDSN(board, str(D / 'r1_power_io.dsn'))
subprocess.run([tools.JAVA, '-jar', tools.FREEROUTING, '--gui.enabled=false', '-de', str(D / 'r1_power_io.dsn'),
                '-do', str(D / 'r1_power_io.ses'), '-mp', '40'], check=True)
pcbnew.ImportSpecctraSES(board, str(D / 'r1_power_io.ses'))
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
board.Save(str(PCB))
print('routed and filled')
```

If this Freerouting version rejects `--gui.enabled=false`, run `$JAVA -jar $R1/pcb/tools/freerouting.jar --help` and use the headless flag it lists.

- [ ] **Step 4: Route and run the test**

Run: `cd $R1 && $KPY pcb/route.py && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_route -v`
Expected: PASS. If clearance violations remain on power nets, widen the spacing in `PLACEMENT` around the reported parts, rerun Tasks 5–6 (`build_pcb.py`, then `route.py`).

- [ ] **Step 5: Archive** (`<N>`=6).

---

### Task 7: Power-path width check

**Files:**
- Create: `$R1/pcb/check_power.py`, `$R1/pcb/tests/ktest_power.py`

**Interfaces:**
- Consumes: routed board, `board_spec.CURRENT_A`, `NETCLASS_POWER`.
- Produces: `check_power.required_width_mm(current_a, oz=2, rise_c=10) -> float`; `check_power.violations(board) -> list[str]`.

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/ktest_power.py`:

```python
import unittest
from pathlib import Path
import pcbnew
from pcb import board_spec as B
from pcb.check_power import required_width_mm, violations

class PowerTests(unittest.TestCase):
    def test_ipc2221_reference_value(self):
        self.assertAlmostEqual(required_width_mm(7.5), 2.43, delta=0.03)   # 2 oz (2.74 mil), 10 C rise, external layer
    def test_all_power_tracks_wide_enough(self):
        b = pcbnew.LoadBoard(str(Path(B.__file__).parent / 'r1_power_io.kicad_pcb'))
        self.assertEqual(violations(b), [])
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_power -v`
Expected: ERROR `No module named 'pcb.check_power'`.

- [ ] **Step 3: Write `pcb/check_power.py`**

```python
"""IPC-2221 external-layer width check for the power nets (2 oz copper, 10 C rise), floor 3 mm (spec section 6)."""
import pcbnew
from pcb import board_spec as B
def required_width_mm(current_a, oz=2, rise_c=10):
    area_mil2 = (current_a / (0.048 * rise_c ** 0.44)) ** (1 / 0.725)
    return area_mil2 / (1.37 * oz) * 0.0254
def violations(board):
    bad = []
    for t in board.GetTracks():
        if t.GetClass() != 'PCB_TRACK' or t.GetNetname() not in B.CURRENT_A: continue
        need = max(3.0, required_width_mm(B.CURRENT_A[t.GetNetname()])) if t.GetNetname() not in ('L_M1', 'L_M2', 'R_M1', 'R_M2') \
            else required_width_mm(B.CURRENT_A[t.GetNetname()])
        if pcbnew.ToMM(t.GetWidth()) + 1e-6 < need: bad.append(f'{t.GetNetname()} {pcbnew.ToMM(t.GetWidth()):.2f} < {need:.2f} mm')
    return bad
```

(Motor output nets carry at most 2.1 A and may use the IPC width; all supply nets use at least 3 mm.)

- [ ] **Step 4: Run it to verify it passes**

Run: `cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_power -v`
Expected: 2 tests OK. If tracks on the motor nets are too narrow, add `'Motor'` (1.2 mm) to the net classes in `make_schematic.py`, regenerate (Tasks 4–6).

- [ ] **Step 5: Archive** (`<N>`=7).

---

### Task 8: Fabrication package

**Files:**
- Create: `$R1/pcb/fab.py`, `$R1/pcb/ORDERING.md`, `$R1/pcb/tests/ktest_fab.py`; generated `$R1/pcb/fab/` (gerbers, drill, `bom.csv`, `r1_power_io_1to1.pdf`, `r1_power_io.step`, `r1_power_io_gerbers.zip`)

**Interfaces:**
- Consumes: routed board, `board_spec.PARTS` (`value`, `mpn`).
- Produces: `fab/r1_power_io.step` (used by Task 9), `fab/bom.csv` (used by Task 10).

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/ktest_fab.py`:

```python
import csv, unittest, zipfile
from pathlib import Path
from pcb import board_spec as B
F = Path(B.__file__).parent / 'fab'

class FabTests(unittest.TestCase):
    def test_gerber_set(self):
        names = zipfile.ZipFile(F / 'r1_power_io_gerbers.zip').namelist()
        for suffix in ('F_Cu.gtl', 'B_Cu.gbl', 'F_Mask.gts', 'B_Mask.gbs', 'F_Silkscreen.gto', 'Edge_Cuts.gm1'):
            self.assertTrue(any(n.endswith(suffix) for n in names), suffix)
        self.assertTrue(any(n.endswith('.drl') for n in names))
    def test_bom_lists_every_part(self):
        rows = list(csv.DictReader(open(F / 'bom.csv')))
        refs = {r for row in rows for r in row['References'].split()}
        self.assertEqual(refs, set(B.PARTS) - {f'H{i}' for i in range(1, 5)})
    def test_step_and_pdf_exist(self):
        self.assertGreater((F / 'r1_power_io.step').stat().st_size, 10_000)
        self.assertGreater((F / 'r1_power_io_1to1.pdf').stat().st_size, 5_000)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_fab -v`
Expected: ERROR (fab directory missing).

- [ ] **Step 3: Write `pcb/fab.py`**

```python
"""Gerbers + drill (zipped), purchase BOM, 1:1 PDF, STEP."""
import csv, subprocess, sys, zipfile
from collections import defaultdict
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, tools
D = Path(__file__).parent; F = D / 'fab'; PCB = str(D / 'r1_power_io.kicad_pcb'); F.mkdir(exist_ok=True)
run = lambda *a: subprocess.run([tools.KICAD_CLI, *a], check=True)
run('pcb', 'export', 'gerbers', '--layers', 'F.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,Edge.Cuts', '-o', str(F) + '/', PCB)
run('pcb', 'export', 'drill', '-o', str(F) + '/', PCB)
run('pcb', 'export', 'pdf', '--layers', 'F.Cu,F.Silkscreen,Edge.Cuts', '-o', str(F / 'r1_power_io_1to1.pdf'), PCB)
run('pcb', 'export', 'step', '--subst-models', '-o', str(F / 'r1_power_io.step'), PCB)
with zipfile.ZipFile(F / 'r1_power_io_gerbers.zip', 'w') as z:
    for p in F.iterdir():
        if p.suffix in ('.gtl', '.gbl', '.gts', '.gbs', '.gto', '.gbo', '.gm1', '.drl'): z.write(p, p.name)
groups = defaultdict(list)
for ref, p in sorted(B.PARTS.items()):
    if not ref.startswith('H'): groups[(p['value'], p['fp'], p['mpn'])].append(ref)
with open(F / 'bom.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['Qty', 'Value', 'Footprint', 'Manufacturer part', 'References'])
    for (value, fp, mpn), refs in sorted(groups.items()): w.writerow([len(refs), value, fp, mpn, ' '.join(refs)])
print('fab outputs in', F)
```

- [ ] **Step 4: Write `pcb/ORDERING.md`**:

```markdown
# Ordering the R1 power & I/O board

Upload `fab/r1_power_io_gerbers.zip` to the fab (JLCPCB or PCBWay) and set:
- Layers 2, material FR-4, thickness 1.6 mm, **outer copper 2 oz**, surface finish **HASL lead-free**, quantity 5.
- No assembly, no stencil (all parts are through-hole).
Before ordering, print `fab/r1_power_io_1to1.pdf` at 100 % (no "fit to page"), check that the Pico, the GY-521, both DRV8874
carriers and the regulator module line up with their footprints, and measure the board length with a ruler (scale check).
Buy the parts in `fab/bom.csv` (Pololu 4035 × 2, JST VH/XH headers, Keystone 3557-2 × 2, G5LE-1, 2N7000 × 5, resistors,
capacitors, LEDs, sockets). You place the order; nothing is ordered automatically.
```

- [ ] **Step 5: Generate and run the tests**

Run: `cd $R1 && $KPY pcb/fab.py && PYTHONPATH=$R1 $KPY -m unittest pcb.tests.ktest_fab -v`
Expected: 3 tests OK.

- [ ] **Step 6: Archive** (`<N>`=8).

---

### Task 9: Robot CAD — board in, old electronics out

**Files:**
- Modify: `$R1/cad/build_robot.py`: the electronics block (the lines that `add('regulator'…)`, `add('driver'…)`, `add('driver_cap'…)`, `add('relay_carrier'…)`, `add('relay'…)`, `add('fuse_1'…)`, `add('fuse_2'…)`, `add('battery_bus'…)`, `add('ground_bus'…)`, `add('controller_carrier'…)`, `add('pico…'…)`, `add('imu'…)`, the J02–J09 connector envelopes, and their `lower_standoff_*` parts and base holes); add the board block after the battery tray.
- Test: `$R1/cad/validation.json` (generated) plus a new validation key `mainboard`.

**Interfaces:**
- Consumes: `pcb/fab/r1_power_io.step`, `pcb/board_outline.json`, `board_spec.board_to_robot`, `Z_PCB_BOTTOM`, `REMOVED_CAD_IDS`.
- Produces: CAD parts `mainboard` (category `pcb`, BOM `E20`), `mainboard_insert_1..4` (BOM `H12`), base pilot holes; validation key `mainboard = {min_gap_mm, max_top_z_mm, ok}`.

- [ ] **Step 1: Find every block to delete**

Run: `grep -n -E "add\('(regulator|driver|driver_cap|relay_carrier|relay|fuse_1|fuse_2|battery_bus|ground_bus|controller_carrier|pico|pico_usb|pico_chip|imu|J0[2-9]|lower_standoff)" $R1/cad/build_robot.py`
Also list their holes: `grep -n -E "lower_mounts|REG\b|REL\b|DRV_X" $R1/cad/build_robot.py`.

- [ ] **Step 2: Write the failing check.** Add to the validation section of `build_robot.py`, just before `report=dict(`:

```python
# Main board: >= 1 mm to every other part (inserts and its own screws exempt), every component below the deck.
mb = shapes.get('mainboard')
mb_gaps = [] if mb is None else [(round(mb.distance(shapes[p['id']]), 2), p['id']) for p in parts
          if p['id'] not in ('mainboard', 'P_base') and not p['id'].startswith(('mainboard_insert', 'W')) and near(mb, shapes[p['id']], 3)]
mainboard_check = dict(present=mb is not None, min_gap_mm=min(mb_gaps)[0] if mb_gaps else None,
                       nearest=min(mb_gaps)[1] if mb_gaps else None, max_top_z_mm=None if mb is None else round(mb.BoundingBox().zmax, 2))
mainboard_check['ok'] = bool(mb is not None and (not mb_gaps or min(mb_gaps)[0] >= 1.0) and mainboard_check['max_top_z_mm'] <= 77.6)
```

and add `mainboard=mainboard_check` to `report` and `mainboard_ok=mainboard_check['ok']` to the summary dict.

- [ ] **Step 3: Run the build to verify the check fails**

Run: `cd $R1 && $CADPY cad/build_robot.py >/tmp/b.log 2>&1; $CADPY -c "import json;print(json.load(open('cad/validation.json'))['mainboard'])"`
Expected: `{'present': False, ..., 'ok': False}`.

- [ ] **Step 4: Delete the old electronics blocks found in Step 1**, together with their `wire(...)` routes W03–W09, W14–W16 and `W_motor_*`.

- [ ] **Step 5: Add the insert pilot holes to the base.** In the base-building section, next to the existing `bcut(f'post {i+1}', ...)` loop:

```python
import json as _json
sys.path.insert(0, str(ROOT))
from pcb import board_spec as BS
MB_HOLES = [BS.board_to_robot(u, v) for u, v in _json.loads((ROOT / 'pcb/board_outline.json').read_text())['holes_uv']]
for i, (x, y) in enumerate(MB_HOLES): bcut(f'mainboard {i + 1}', cyl(x, y, 28, 2.1, 6))   # Ø4.2 pilots for M3 heat-set inserts
```

- [ ] **Step 6: Add the board and its bosses** after the battery-tray block:

```python
# R1 power & I/O board (home-robot-r1/pcb): routed KiCad board exported as STEP, on four 6 mm printed bosses with M3 inserts.
mb = cq.importers.importStep(str(ROOT / 'pcb/fab/r1_power_io.step')).val()
mb = mb.mirror('XZ').translate((BS.BOARD_X0, BS.BOARD_Y0, BS.Z_PCB_BOTTOM))   # board (u, v) -> robot (x, y); see the check below
add('mainboard', 'R1 power & I/O board with modules', mb, 'pcb', 'E20', 'lower', check=True,
    note='Ordered 2-layer PCB (pcb/ORDERING.md): Pico, GY-521, 2 × DRV8874, regulator, fuses, relay; see pcb/BRINGUP.md.')
for i, (x, y) in enumerate(MB_HOLES):
    add(f'mainboard_insert_{i + 1}', 'M3 heat-set insert and 6 mm boss under the main board', cyl(x, y, 33, 4, 6), 'metal', 'H12', 'lower')
```

Check the STEP orientation once with `print(mb.BoundingBox().xmin, xmax, ymin, ymax, zmin)`. It must span x ≈ 14–98, y ≈ −85–85 and z ≥ 39, and the Pico (`PLACEMENT['U1A']`) must sit at robot `board_to_robot(52, 40)` ≈ (66, 45). If y comes out mirrored, remove `.mirror('XZ')`.

- [ ] **Step 7: Rebuild and verify**

Run: `cd $R1 && $CADPY cad/build_robot.py >/tmp/b.log 2>&1; echo $?; $CADPY -c "import json;v=json.load(open('cad/validation.json'));print(v['mainboard'], v['clash_scan_empty'], v['printability']['all_pass'])"`
Expected: exit 0, `mainboard.ok True`, `clash_scan_empty True`, `all_pass True`.

- [ ] **Step 8: Archive** (`<N>`=9).

---

### Task 10: Harness and BOM

**Files:**
- Create: `$R1/pcb/update_bom.py`, `$R1/pcb/tests/test_bom.py`
- Modify: `$R1/cad/build_robot.py` (new `wire(...)` routes), `$R1/parts-and-connections.json` (through `update_bom.py`)

**Interfaces:**
- Consumes: `board_spec.PLACEMENT` + `board_to_robot` (connector positions), `pcb/fab/bom.csv`.
- Produces: harness routes `W_bat`, `W_rocker`, `W_jetson`, `W_cliff_1..4`, `W_bumper_l/r`, `W_arm`, `W_toggle`, `W_pico_usb`; updated BOM ids.

- [ ] **Step 1: Write the failing test** `$R1/pcb/tests/test_bom.py`:

```python
import json, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
class BomTests(unittest.TestCase):
    def setUp(self): self.ids = {p['id'] for p in json.loads((ROOT / 'parts-and-connections.json').read_text())['parts']}
    def test_old_electronics_removed(self):
        self.assertEqual(self.ids & {'E06', 'E10', 'E11', 'C13', 'P06'}, set())
    def test_board_items_present(self):
        self.assertTrue({'E20', 'E21', 'E22', 'E23', 'C18', 'C19'} <= self.ids)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd $R1 && $CADPY -m unittest pcb.tests.test_bom -v`
Expected: FAIL (old ids still present).

- [ ] **Step 3: Write `pcb/update_bom.py`**

```python
"""Replace the perfboard / WAGO / MDD3A / inline-fuse items with the main board and its parts."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; F = ROOT / 'parts-and-connections.json'
d = json.loads(F.read_text())
d['parts'] = [p for p in d['parts'] if p['id'] not in ('E06', 'E10', 'E11', 'C13', 'P06')]
new = [
    dict(id='E20', category='Control board', item='R1 power & I/O board, 2-layer 2 oz PCB (pcb/fab)', quantity=1, unit='board',
         status='Order (5 pcs)', new_cost_usd=8, source='pcb/ORDERING.md', notes='Hand-soldered, through-hole. See pcb/BRINGUP.md.', model='mainboard'),
    dict(id='E21', category='Drive', item='Pololu DRV8874 single brushed DC motor driver carrier, item 4035', quantity=2, unit='carriers',
         status='Buy', new_cost_usd=19.9, source='https://www.pololu.com/product/4035', notes='Plug into the main board sockets.', model='mainboard'),
    dict(id='E22', category='Control board', item='Board components per pcb/fab/bom.csv (resistors, 2N7000 × 5, LEDs, capacitors, 1N5819)',
         quantity=1, unit='set', status='Buy', new_cost_usd=6, source='pcb/fab/bom.csv', notes='', model='mainboard'),
    dict(id='E23', category='Power', item='Keystone 3557-2 PCB blade-fuse holder', quantity=2, unit='holders', status='Buy', new_cost_usd=3,
         source='', notes='F1 7.5 A main, F2 5 A motor (fuses: P07).', model='mainboard'),
    dict(id='C18', category='Connections', item='JST VH 2-pin board headers B2P-VH (J1-J3) and mating sets', quantity=3, unit='sets',
         status='Buy', new_cost_usd=4, source='', notes='Battery adapter lead, master rocker pair, Jetson lead.', model='mainboard'),
    dict(id='C19', category='Connections', item='JST XH board headers: 5-pin × 4, 2-pin × 4, 4-pin × 1, with housings and contacts',
         quantity=1, unit='set', status='Buy', new_cost_usd=6, source='', notes='Cliff sensors, bumpers, arm button, motor toggle, spare I2C.', model='mainboard'),
]
d['parts'] += [p for p in new if p['id'] not in {q['id'] for q in d['parts']}]
F.write_text(json.dumps(d, indent=2, ensure_ascii=False))
print('BOM updated:', len(d['parts']), 'items')
```

- [ ] **Step 4: Rewire the harness in `build_robot.py`.** Replace the deleted routes with these, after the board block. Each endpoint is `board_to_robot(PLACEMENT[ref])` at z 43, run with the existing `wire(id, name, points, radius, kind, bom)` helper:

```python
def jpt(ref, dz=4):   # connector mating point on the board (robot frame)
    u, v, _ = BS.PLACEMENT[ref]; x, y = BS.board_to_robot(u, v); return (x, y, BS.Z_PCB_BOTTOM + BS.PCB_T + dz)
wire('W_bat', 'Battery adapter lead → J1', [(-96, 0, 47), (-40, 0, 47), jpt('J1')], 2, 'power', 'C18')
wire('W_rocker', 'J2 ↔ lid master rocker (pair)', [jpt('J2'), (jpt('J2')[0], 30, 70), (-104, 30, 118), (-104, 4, 121)], 2, 'power', 'C18')
wire('W_jetson', 'J3 → Jetson barrel lead', [jpt('J3'), (jpt('J3')[0], -30, 70), (JX - 46, -25, 96)], 2, 'power', 'C18')
for i, cid in enumerate(('cliff_3', 'cliff_4', 'cliff_1', 'cliff_2')):   # J6..J9 = front-left, front-right, rear-left, rear-right
    b = next(p for p in parts if p['id'] == cid)['bounds']; c = ((b[0] + b[3]) / 2, (b[1] + b[4]) / 2, b[5] + 2)
    wire(f'W_cliff_{i + 1}', f'Cliff sensor {i + 1} → J{6 + i}', [c, (c[0], c[1], 60), (jpt(f"J{6 + i}")[0], jpt(f"J{6 + i}")[1], 60), jpt(f'J{6 + i}')], 1.3, 'signal', 'C19')
```

```python
wire('W_arm', 'Arm button → J12', [(-80, -70, 128), (-80, -96, 118), (jpt('J12')[0], -96, 70), jpt('J12')], 1.3, 'signal', 'C19')
wire('W_toggle', 'Motor toggle loop → J13', [(-85, 67, 128), (-85, 96, 118), (jpt('J13')[0], 96, 70), jpt('J13')], 1.3, 'signal', 'C19')
wire('W_pico_usb', 'Pico USB → Jetson', [jpt('U1A', 12), (jpt('U1A')[0], -96, 70), (JX - 5, -96, 94), (JX - 5, -70, 94)], 2, 'signal', 'C02')
```

In the existing `wire('W_bumper_left', ...)` and `wire('W_bumper_right', ...)` calls (they already pass the base lead holes), replace the final point with `jpt('J10')` and `jpt('J11')`. The build's wire clash check confirms every route.

- [ ] **Step 5: Run the BOM update, rebuild, and run the tests**

Run: `cd $R1 && $CADPY pcb/update_bom.py && $CADPY cad/build_robot.py >/tmp/b.log 2>&1; echo $?; $CADPY -m unittest pcb.tests.test_bom -v && $CADPY -c "import json;v=json.load(open('cad/validation.json'));print(v['wire_plate_clashes'], v['clash_scan_empty'], v['summary'].get('bom_consistency_ok'))" && $CADPY -c "import json;print(len(json.load(open('cable-cut-list.json'))))"`
Expected: exit 0; 2 BOM tests OK; `[] True True`; the cable list has fewer routes than before (29), about 12 plus USB.

- [ ] **Step 6: Archive** (`<N>`=10).

---

### Task 11: Simulator sync, bring-up guide, final checks

**Files:**
- Create: `$R1/pcb/BRINGUP.md`
- Modify (generated): `$SIM/Content/RobotData/robot-parameters.json`, `$SIM/Content/RobotData/assembly-meshes.json`, `$R1/design-review.html`

**Interfaces:**
- Consumes: the rebuilt CAD (Tasks 9–10).
- Produces: synced simulator data; the bring-up guide.

- [ ] **Step 1: Write `pcb/BRINGUP.md`**:

```markdown
# R1 power & I/O board — bring-up

1. Bare board: with a multimeter in continuity mode, check that BAT+, VBAT_SW, VMOT, 5V and 3V3 are NOT shorted to GND (test pads TP1-TP5).
2. Solder the low parts first (resistors, diode, 2N7000s, LEDs), then sockets and headers, fuse holders, relay, capacitors (C1 polarity: stripe = minus).
3. No modules, no fuses: connect the battery through J1 — nothing should light. Insert F1 and close the lid rocker: LED1 (VBAT_SW) on; TP1 reads the battery voltage.
4. Regulator: fit the 5 V regulator (either the P03 module on U5A/U5B or a D24V50F5 in U5C, never both). LED2 on; TP3 = 5.0–5.2 V. Leave the Jetson unplugged.
5. Motor branch: insert F2, close the motor toggle (J13): the relay clicks, LED3 on, TP2 = battery voltage. Open the toggle: LED3 off.
6. Pico: fit it, power it from the Jetson USB only. TP4 = 3.3 V. Run the firmware's pin self-test: GP16 reads LOW with the toggle closed, HIGH open; GP14/GP15 read LOW with the bumpers released (NC to GND), HIGH pressed; GP26 ADC ≈ battery × 33/133.
7. IMU and cliff sensors: fit the GY-521, plug the four cliff sensors, scan I2C: 0x68 plus the four ToF sensors after their XSHUT address assignment (0x30-0x33).
8. Drivers: fit both DRV8874 carriers with the wheels off the floor. Firmware sets GP18 (nSLEEP) high, drives GP6/GP7 then GP8/GP9 with a low PWM: each wheel turns both ways; GP19 (nFAULT) stays HIGH; GP27/GP28 read a current that rises when the wheel is braked by hand.
9. Plug the Jetson lead (J3) last and boot the Jetson.
```

- [ ] **Step 2: Sync the robot data into the simulator and run the controller tests**

Run: `cd $SIM && .venv/bin/python Scripts/sync_robot_data.py --cad $R1/cad && cd tests && PYTHONDONTWRITEBYTECODE=1 ../.venv/bin/python -B -m unittest test_autonomy test_lidar test_exploration test_mono_obstacles test_floor_parallax test_whole_house_pacing 2>&1 | tail -3`
Expected: the sync prints `CAD synced ... COM ... behind the axle`; `OK`. If the center of mass moved forward past the sync guard (−15 mm), report it to the user. Do not add ballast without asking.

- [ ] **Step 3: Regenerate the review page**

Run: `cd $R1 && $CADPY cad/rebuild_review.py`
Expected: `design-review.html` regenerated without errors.

- [ ] **Step 4: Final spec check.** Confirm each acceptance item in spec section 9 and list the evidence (file + value):
  - ERC 0: `pcb/erc.json`.
  - Net cross-check: `test_schematic`.
  - DRC 0 / fully routed: `pcb/drc.json`.
  - Power widths: `ktest_power`.
  - CAD clash scan empty and printability all-pass: `cad/validation.json`.
  - Simulator sync and tests: Step 2 output.

- [ ] **Step 5: Archive** (`<N>`=11).
