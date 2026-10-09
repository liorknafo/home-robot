# R1 power & I/O board under the Jetson ("bay board", 4 layers): design spec amendment

Date: 2026-10-07. Status: layout chosen by the user ("B"). This spec is waiting for the user's review.
Robot: home-robot-r1.

This spec amends the low-profile spec (`2026-10-06-r1-low-profile-design.md`).
- It replaces that spec's **Main board** and **5 V regulator (P03)** rows (§2).
- It replaces its connector-edge list (§4) and its board acceptance line (§6).
- It changes the original board spec (`2026-10-04-r1-power-io-board-design.md`) from "one 2-layer PCB" to one 4-layer PCB.

Everything else in both specs stays in force.

## 1. Why

The 50 × 178 mm strip between the battery and the motors did not route on 2 layers:
- L6 left 12–21 connections unrouted.
- The L5b rework got down to 1–3 unrouted per run, over 24 router runs on the final placement.

The cause was the shape. The strip is a 50 mm sliver, and part of it lies under the Jetson, where parts must be ≤ 12 mm. As a result, every power trunk cuts it across its whole width. On 4 layers the strip costs $30.50 for 5 boards.

The user then pointed out three things:
- The board need not sit on the strip.
- There is enough room under the Jetson.
- Nothing has been printed or ordered yet.

A fit study (spike W4L, `task-W4L-report.md` in the SDD folder) placed every part on one board under the unchanged Jetson. The board is within 100 × 100 mm, so JLCPCB's small-board price applies: 4 layers for $8 per 5. The user chose this layout over moving the Jetson sideways.

## 2. Decisions

| Topic | Decision | Why |
|---|---|---|
| Board location | **At base level, under the Jetson, between the motors.** The nominal outline spans robot x −66.5..7.5 and y −50..50 (bounding box 74 × 100 mm). Board bottom z 35.4, top z 37.0. At that height a standard M2.5 × 20 mm standoff above the board keeps the Jetson at z 57. THT leads are trimmed to ≤ 1.4 mm, leaving 1.0 mm of air above the base. The outline comes from the real CAD meshes: ≥ 1.5 mm to every fixed part, the same rule as L4. | The space under the Jetson (z 33 → 54.77) is empty except for the motors and the Jetson standoffs. The 28 mm band behind the Jetson (x < −38.5) has full height up to the lid. |
| Outline shape | Full width for x −66.5..−14. A tongue at \|y\| ≤ 33.5 for x −14..7.5, between two notches round the motor bodies (motor x ±12.5, \|y\| ≥ 35, plus 1.5 mm). The two rear Jetson standoffs at (−33, 40) and (−33, −46) pass through the board, or clamp it (§2, Mounting). The front Jetson standoffs at x 25 lie ahead of the board. | Motors and standoffs are fixed. |
| Layers | **4 layers, 1.6 mm.** F.Cu: signals + SMD. In1: solid GND. In2: power pours (VBAT_SW, VMOT, 5V, 3V3) + signals. B.Cu: signals. The user's 4-layer request, at ≤ 100 × 100 mm. | A solid GND layer makes the return-path rules easy. Routing is no longer the blocker. |
| Copper weight | Use the quote's default options (JLCPCB 4-layer default: 1 oz outer, 0.5 oz inner; confirm when ordering). Power paths are sized from IPC-2221 for those weights at ≤ 10 °C rise. Parallel copper on F.Cu / In2 / B.Cu with via stitching counts. Heavier copper changes the price, so it needs the user's approval. | The R4 widths were computed for 2 oz, so they are recomputed. The power block is compact (J1 → F1 → J2 → F2 → K1 → C1 → drivers, all within about 60 mm). |
| Height zones | **Under the carrier** (x ≥ −38.5): every part top, including a mated plug and its wire bend, ≤ z 54.77 − 1.5, i.e. ≤ 16.27 mm above the board top. **Behind the carrier** (x < −38.5): part tops ≤ lid underside − 3 mm (z 97.5). The old "carrier band" rule (BAND_PLAN_MARGIN / BAND_PLAN_EXEMPT) is retired. | The carrier PCB bottom is at z 57, and its lowest underside parts are 2.23 mm below that. |
| What goes where | **Behind the carrier** (exposed with the lid off): F1, F2, K1, C1; the Pico, with its USB plug out of the left edge into the old strip-end space (x ≈ −54..−42, y 50..70), pluggable from above; J1 battery, J2 rocker, J13 toggle loop, J12 arm button, J6 and J7 rear cliffs, on the rear edge.<br>**Under the carrier:** both DRV8874 carriers, the GY-521, J4 / J5 on the motor-notch edges, J10 / J11 bumpers on the side edges, J8 / J9 front cliffs and J3 Jetson 5 V on the tongue's front edge, U5A / U5B facing the P03, and the spares J14 / J15.<br>The spike's placement (`handoff-scratch/w4l/bay1_full.py`) is the starting point. L5 may change it within these rules. | The connectors you plug often, the fuses and the Pico USB stay reachable without lifting the Jetson. |
| Connectors under the carrier | Each must leave its cable a path in the 17 mm gap. Either use a right-angle (side-entry) housing or header, or prove that the vertical part's mated height plus a 90° wire bend fits the 16.27 mm zone. The cable must leave at a board edge, outward. They are plugged before the Jetson is fitted, as already accepted for the right motor lead. | All current connectors are vertical. With a plug on, that leaves about 7 mm for the wire bend under the carrier. |
| Motor cables | The motors' own 6-wire cables (factory 200 mm) plug straight into J4 / J5 on the notch edges. The W_motor harness and the motor-header joints are dropped. | The motors are right beside the notches. |
| 5 V regulator (P03) | Turned 90° in the front bay: x 9..47, \|y\| ≤ 31, z 33–48, held with foam tape and new cable-tie slots in P_base. Its leads to U5A / U5B are about 10 mm. Keep its trimpot reachable for the first setting, with the Jetson off. | No legal flat spot is left at base level outside the board. |
| Mounting | 4 × M3 holes into heat-set inserts in P_base, on 2.4 mm printed PETG spacers. The spike positions (−19.5, ±38), (−5.5, 24), (−7, −14.5) are the starting point. **The band behind the carrier must also be supported.**
- The board is clamped in the two rear Jetson standoff stacks: base → 2.4 mm spacer → board → M2.5 × 20 mm standoff → Jetson. The Jetson stays at z 57.
- Printed bearing ribs on P_base sit under the rear-edge connector row, clear of THT pins, to take the plug insertion force.
- Extra M3 holes in the band are optional, if they fit. Each M3 hole keeps the 4.5 mm GND-only washer keep-out (R17). The 4 old strip inserts are removed. | In the spike all 4 holes sit at x ≥ −19.5. That leaves the plug-heavy rear edge (battery, rocker, switches) 47 mm from any fixing. |
| Status LEDs and test pads | Place them where they can be seen or reached with the lid off (behind the carrier) if they fit without breaking the rules there. Otherwise place them under the carrier and record that in BRINGUP. | Bring-up happens with the Jetson off, when everything is exposed. In service, the LEDs under the Jetson are hidden. |
| Pico GPIO map | PICO_GPIO in board_spec stays the single map. L5 may remap it within the L5b brief's hardware constraints (I2C pairs, ADC only on GP26–28, PWM slice A / B). parts-and-connections.json follows. | No firmware exists yet. |
| Cost | $8 per 5 boards (4 layers, ≤ 100 × 100), plus a stencil, plus about $31 DHL. The user orders; agents never order. | |

## 3. Rules carried over and changed

**Unchanged:**
- The circuit: NETS / PARTS, U5C removed.
- LP3/LP4 assembly: reflow on top only; ≥ 1 mm SMD to SMD; ≥ 2 mm from an SMD pad to THT pads; silk values and polarity; F.Paste plus stencil; order reflow → THT → modules.
- R13 thermal reliefs.
- R16: Motor class with 1.0 mm clearance.
- R17: vias ≥ 0.5 mm from other-net pads; washer keep-outs.
- LP12: one tented GND via per top-side SMD GND pad, now to the In1 plane.
- Zero GND tracks.
- Placement parity with board_spec.
- route.py writes the board only on a clean result.
- Module sockets unmirrored.
- C1 within 30 mm of both drivers' VIN pins.

**Changed:**
- **R4:** the net-class widths are recomputed for the ordered copper (§2). The POWER_ZONES trunk check applies to the pours on F.Cu / In2.
- **R15:** return paths are measured on In1. The limits stay at ≤ 1.05× for the power loops and ≤ 1.2× for the signal returns. Add a check that fails on any slot in In1 longer than 10 mm. "Longest router B.Cu run ≤ 40 mm" is dropped, because In1, not B.Cu, carries the returns.
- **LP7 and the strip connector-edge list are replaced** by §2 "What goes where".
- **USB plug path:** the Pico USB path is tested at the board's left edge (y +50), clear up to the plug position.
- **Obsolete:** the strip's PREROUTES, POWER_ZONES, HOP_VIAS and CELL_GRIDS. route.py, zone_check and the tests are re-targeted to 4 layers.

## 4. CAD changes (build_robot.py)

**Board and mounting:**
- Add the bay board at z 35.4, on its spacers and inserts, with the real KiCad board in place.
- Clamp the board in the rear Jetson standoff stacks, with M2.5 × 20 mm standoffs above the board and longer base screws. The Jetson stays at z 57.
- Add the bearing ribs on P_base under the rear connector row.

**Removed:**
- the strip board, its modules and placeholders;
- its spacers and screws, and the `insert_strip_*` inserts;
- the motor-header joints and the W_motor_* harness.

**P03:** turned 90° into the front bay, with tie slots. Re-check it against the bumper riders B at full bumper travel.

**Clash scan, with the real meshes:**
- the board parts under the carrier;
- the real carrier underside lead tails (not in the simplified mesh; measure them at bring-up);
- the Pico USB plug;
- every under-carrier cable path.

**Also:**
- Regenerate the W* cable runs.
- Update REMOVED_CAD_IDS.
- Update the mass records for the board and modules.
- Regenerate design-review.html.

## 5. Harness changes (L10)

Rough lengths are in the spike report, §8.
- The net harness change is about +169 mm.
- The battery, rocker and switch leads grow by 40–95 mm.
- The bumper leads shorten by 10 mm.
- The Pico USB cable shortens by 25 mm.
- The P03 leads shrink to about 10 mm.
- The motor harness is gone.

**BOM:**
- Add: inserts, M2.5 × 20 mm nylon F-F standoffs for the rear pair with longer screws, and right-angle connectors where L5 chooses them.
- Drop: the strip spacers, the W_motor parts and the D24V50F5.

## 6. Acceptance

**Board:**
- The outline comes from the CAD, with ≥ 1.5 mm to real geometry.
- DRC 0 errors / 0 unconnected on 4 layers.
- ktest_pcb is green, adapted to the height zones, edges, USB path and washers.
- ktest_route is green:
  - R15 on In1 within limits, with no In1 slot over 10 mm;
  - R16 and R17;
  - no GND tracks, plus the LP12 vias;
  - the power trunks on their pours.
- The pure tests are green.
- L7 power check: IPC-2221 at ≤ 10 °C rise for the ordered copper.

**CAD:**
- The build exits 0.
- The clash scan is empty, including the board parts, plugs and cable paths under the carrier.
- Printability passes for every printed part.
- COM x ≤ −15 mm, nominal and worst case. The spike estimate is −18.25 / −15.70; re-check it after the harness moves.
- The robot top is unchanged at 155.15 mm (≤ 160).

**Simulator:** re-synced from the new mass records. The low-profile spec §5 acceptance is unchanged.

## 7. Risks

1. **The band behind the carrier is nearly full.** About 98.5 of 100 mm is used along y. If a fuse holder, the relay or C1 grows, or support holes are needed there, use one of these fall-backs:
   - C1 as an SMD can (Ø10 × 10.5);
   - C1 laid flat;
   - the full 100 × 100 board, with the P03 on a printed shelf over a motor (spike variant B-ii, COM −18.97 / −16.39).
2. **Height under the carrier.** The DRV and IMU sockets have 2.27 mm to the 16.27 mm limit. The carrier's real lead tails are not in the CAD mesh. BRINGUP measures the gap before the Jetson goes on.
3. **COM margin.** The worst case is about 0.7 mm inside the limit. The wiring record (0.1 kg) has not moved yet. If the margin is lost, move the board group rearward or ask the user about ballast. Agents never add ballast on their own.
4. **Service access.** Connectors, LEDs and test pads under the carrier need the Jetson off. This is accepted for fixed harness connections only.
5. **Cooling under the Jetson:** the drivers and the P03 sit under the carrier. The DRV8874 dissipation at the robot's motor current is small. L7 lists it, and BRINGUP's lid-on tegrastats test covers the Jetson.

## 8. What this supersedes in the plans

- **Low-profile plan, tasks L4–L11:**
  - The board tasks L4–L8 (outline, placement, routing, power check, fab) are redone for the bay board.
  - L9 (CAD), L10 (harness / BOM) and L11 (sim, BRINGUP, viewer) keep their deliverables but follow this spec.
  - A revised plan replaces them.
- **Strip work:** the L4–L6 and L5b commits (b61db36 … 751f0fa) stay in git history as superseded. The circuit, schematic, footprints, rules, test framework and the routing pipeline (route.py, zone_check) are reused.
- **Paused branches:** the 4-layer split spike (worktree `spike-4L-split`) and the 2-layer strip retry (L5b) are dropped.
