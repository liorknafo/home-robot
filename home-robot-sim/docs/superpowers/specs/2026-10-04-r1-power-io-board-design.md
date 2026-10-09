# R1 power & I/O board — design spec

Date: 2026-10-04 · Status: design approved in conversation, spec awaiting review · Robot: home-robot-r1 (round, centre-turning)

## 1. Goal

Replace most of the robot's point-to-point wiring (29 harness runs, two WAGO buses, two inline fuse holders, two
hand-soldered perfboards) with **one ordered 2-layer PCB** that carries the existing plug-in modules and gives every
off-board device one keyed connector.

Out of scope: the simulator (it models behaviour, not electronics), the drive/caster redesign (100 mm wheels, raised
chassis — separate task; the board sits on the base and moves with it), and firmware beyond the optional reading of the
new driver lines (section 4).

## 2. Decisions (agreed in conversation)

| Topic | Decision |
|---|---|
| Build | Ordered PCB (JLCPCB / PCBWay class fab), 2 layers, 2 oz copper, 1.6 mm FR-4, lead-free HASL |
| Integration | Plug-in modules; all other parts through-hole so the user hand-solders the board |
| Boards | One main board in the lower bay |
| Motor driver | Two Pololu DRV8874 single brushed-DC driver carriers (item 4035) replace the Cytron MDD3A |
| Connectors | JST-VH for power, JST-XH for signals (both already in the parts list), 2.54 mm headers for the motor cables |

## 3. Architecture

```
Battery 7.2 V NiMH ──J1──F1 7.5 A──J2 (lid master rocker)──► VBAT_SW (switched battery rail)
    VBAT_SW ─► regulator module ─► 5V rail ─► J3 Jetson power lead, encoder 5 V, relay coil loop, LEDs
    VBAT_SW ─► F2 5 A ─► relay K1 (NO contact) ─► VMOT ─► 470 µF ─► DRV8874 U3 (left) / U4 (right) ─► J4 / J5 motors
    VBAT_SW ─► 100 k / 33 k divider + 100 nF ─► Pico GP26 (ADC0)
    5V ─J13 (lid motor toggle)─► K1 coil (1N5819 flyback) ; coil+ ─► 2N7000 ─► GP16 (motor power present)
Pico (socket, USB to Jetson only) 3V3 OUT ─► IMU socket, 4 cliff connectors, spare I2C, pull-ups
```

Board zones: power section (J1, F1, F2, K1, regulator, drivers, bulk capacitor, J4/J5) at one end; logic section (Pico,
IMU, level shifters, signal connectors) at the other. Solid ground plane on the bottom layer; motor and regulator return
currents do not cross the logic area.

## 4. Pico pin map

Existing assignments are unchanged (firmware compatible). New lines use former spares.

| Pins | Function | Board circuit |
|---|---|---|
| GP0, GP1 | Left encoder A, B | 2N7000 inverting stage each: gate from 5 V encoder signal, 100 kΩ gate pull-down, 10 kΩ drain pull-up to 3V3 |
| GP2, GP3 | Right encoder A, B | same |
| GP4 / GP5 | I2C SDA / SCL | 4.7 kΩ pull-ups to 3V3 on the board. The IMU and ToF modules add their own pull-ups in parallel: measure SDA/SCL to 3V3 at bring-up; if the combined value is below 1 kΩ, leave the board pull-ups unpopulated |
| GP6, GP7 | Left driver IN1, IN2 | 10 kΩ pull-downs (motor off while the Pico boots) |
| GP8, GP9 | Right driver IN1, IN2 | 10 kΩ pull-downs |
| GP10–GP13 | Cliff sensor XSHUT 1–4 | direct (driven low / released) |
| GP14 / GP15 | Bumper left / right | switch COM–NC to GND, internal pull-up: open wire reads as pressed |
| GP16 | Motor power present | 2N7000 on relay coil+ (LOW = motor power on) |
| GP17 | Arm button | normally-open to GND, internal pull-up |
| GP18 | Drivers nSLEEP (both) | 100 kΩ pull-down: drivers asleep until firmware enables them **(new)** |
| GP19 | Drivers nFAULT (wired-OR) | 10 kΩ pull-up to 3V3 **(new)** |
| GP26 (ADC0) | Battery voltage | 100 kΩ / 33 kΩ divider + 100 nF |
| GP27 (ADC1) | Left motor current | driver current-sense output **(new)** |
| GP28 (ADC2) | Right motor current | driver current-sense output **(new)** |
| GP20–GP22 | Spare | J15 spare header |

Driver mode pins are tied so the two-input (IN/IN) control scheme of the MDD3A is kept. The DRV8874 carrier's exact pin
names, mode-pin levels and current-sense scale are taken from Pololu's item 4035 documentation before schematic capture;
the current-sense output must stay within 0–3.3 V at the motors' stall current.

## 5. Connectors

| Ref | Function | Type | Pins |
|---|---|---|---|
| J1 | Battery in | JST-VH 2 (keyed). A short adapter lead (Tamiya plug to VH-2 housing, 18 AWG) connects the battery; the battery's own connector stays intact | 1 BAT+, 2 GND |
| J2 | Lid master rocker | JST-VH 2 | 1 F1 out, 2 VBAT_SW |
| J3 | Jetson 5 V lead | JST-VH 2 (to the existing barrel lead) | 1 5V, 2 GND |
| J4 / J5 | Left / right motor | 1×6 male 2.54 mm (mates the motor's own cable) | M+, M−, encoder GND, encoder 5 V, A, B (vendor wire order) |
| J6–J9 | Cliff sensors 1–4 | JST-XH 5 | 3V3, GND, SDA, SCL, XSHUT |
| J10 / J11 | Bumper left / right | JST-XH 2 | GP14 / GP15, GND |
| J12 | Arm button | JST-XH 2 | GP17, GND |
| J13 | Lid motor-power toggle loop | JST-XH 2 | 5V, relay coil+ |
| J14 | Spare I2C | JST-XH 4 | 3V3, GND, SDA, SCL |
| J15 | Spare GPIO | 1×6 2.54 mm | 1 GP20, 2 GP21, 3 GP22, 4 3V3, 5 5V, 6 GND |
| U1 | Pico | 2 × 1×20 female sockets | USB stays a cable to the Jetson; no board feed to VBUS |
| U2 | MPU-6050 (GY-521) | 1×8 female socket | VCC from 3V3, AD0 to GND (address 0x68) |
| U3 / U4 | DRV8874 carriers | female sockets on the carrier's two pin rows | per Pololu pinout |
| U5 | 5 V regulator | footprint A: pads/screw terminals VIN+, VIN−, VOUT+, VOUT− plus mounting holes for the existing module (P03, measured); footprint B: Pololu D24V50F5 header (P04) |

Every off-board connector sits on a board edge facing its cable: motor headers toward the motors, cliff and bumper
connectors toward the front/rear, power connectors aft toward the battery.

## 6. Power design

- Rails: VBAT_SW 6.0–8.7 V; VMOT (relay-switched, fused 5 A); 5V from the regulator (Jetson up to about 4 A plus about 0.3 A
  for encoders, relay and LEDs); 3V3 from the Pico (IMU + 4 ToF ≈ 100 mA of its 300 mA).
- Fuses: board-mount blade-fuse holders (standard automotive blades): F1 7.5 A main, F2 5 A motor branch.
- Current paths: 7.5 A on VBAT/GND and the 5V-to-J3 path; 5 A on VMOT. Widths are computed from IPC-2221 for 2 oz copper with
  at most 10 °C rise and then doubled on both layers with via stitching; the minimum on any power path is 3 mm.
- Relay K1: Omron G5LE-1 DC5 (existing part P05), 1N5819 across the coil. Toggle off, broken loop or relay failure leaves
  motors unpowered while the Jetson stays on.
- Bulk 470 µF / 25 V at the drivers' supply pins; the existing 100 nF capacitors stay on the motor terminals.
- Reverse polarity: prevented by the keyed J1 only (no series diode: it would waste power at these currents).
- Bring-up aids: three LEDs (VBAT_SW, 5V, VMOT) with resistors; test pads on VBAT_SW, VMOT, 5V, 3V3 and GND.

## 7. Mechanical integration

- Location: lower bay, front section, between the motors and the camera tower: board envelope about 84 × 170 mm at robot
  x 14–98 mm, |y| ≤ 85 mm, notched around anything the clash scan finds (cliff mounts at (105, ±72), camera-tower feet at
  x ≥ 100).
- Mounting: four M3 screws into heat-set inserts in printed bosses on P_base (6 mm standoff height).
- Height: everything stays below the deck underside (z 78); tallest part is the regulator module (about 30 mm).
- Removed from the robot: MDD3A (E06), both perfboards (E10, E11), both WAGO buses (C13), both inline fuse holders (P06),
  the old regulator / relay bays, the deck controller carrier and their wiring. The freed rear bays and deck area stay free.
- CAD: the board becomes a modeled part in build_robot.py (PCB plate plus module and connector envelopes, imported board
  STEP where available); the clash scan, printability, BOM (parts-and-connections.json) and cable list are regenerated.
  Expected harness after the change: about 12 short runs (battery, master rocker pair, Jetson lead, two factory motor
  cables, four cliff sensors, two bumpers, arm button, motor toggle) plus the USB cables.

## 8. Deliverables

1. KiCad 9 project: schematic and PCB (tools installed only after the user approves each download: KiCad, the
   Freerouting autorouter and its Java runtime).
2. Fabrication package: Gerbers, drill file, purchase BOM with supplier part numbers, ordering notes (2-layer, 2 oz,
   1.6 mm, lead-free). The user places the order; nothing is ordered or paid for automatically.
3. A 1:1 printable PDF of the board for checking module footprints before ordering.
4. Updated robot CAD (section 7), regenerated BOM and cable list, design-review page.
5. Bring-up guide: continuity checks, first power-up without modules (LEDs, test pads), then modules one at a time.

## 9. Acceptance checks

- Schematic ERC: 0 errors. A script cross-checks every net against the tables in sections 4–5 (100 % match).
- PCB DRC: 0 errors with the fab's 2-layer rules plus margin (traces and clearance ≥ 0.25 mm on signals).
- Power-path widths ≥ the computed values in section 6 (script check).
- Robot CAD: clash scan empty with the board and modules; printability gate still all-pass.
- Simulator: unchanged behaviour; the robot data sync still runs (masses and centre of mass update).

## 10. Risks and items to verify

- Existing regulator module (P03): pinout, hole pattern and height must be measured; footprint B (D24V50F5) is the fallback.
- DRV8874 carrier pinout, mode-pin defaults and current-sense scale (Pololu documentation) — verified before schematic capture.
- Motor stall current vs the drivers' 2.1 A continuous rating (expected well below at 7.2 V).
- 2.54 mm motor headers carry motor current (same as today's retained mates); acceptable at these currents.
- Board cost: 5 boards at this size with 2 oz copper are roughly USD 25–50 plus shipping; drivers 2 × USD 9.95; fuse
  holders and connectors about USD 10; the MDD3A and WAGO blocks are no longer needed.
