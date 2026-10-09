# R1 Low-Profile Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Lower the robot to ≤ 160 mm by removing the deck: the Jetson goes on base standoffs over the motors, the power & I/O board becomes a 48 × 178 mm strip between the battery and the motors, and the upper structure drops about 43 mm. Then re-verify the CAD, the board, the simulator and mapping.

**Architecture:**
- **CAD:** `home-robot-r1/cad/build_robot.py` (CadQuery) stays the single robot model. Its validation (`cad/validation.json`) and the simulator sync (`home-robot-sim/Scripts/sync_robot_data.py`) are the gates.
- **Board:** `home-robot-r1/pcb/` keeps its generators. Only the outline, placement, routing and fab outputs are regenerated, against the new CAD.

**Tech stack:**
- CadQuery 2.8: CAD venv `$CADPY`.
- KiCad 9.0.9: `$KPY`, `$KCLI`.
- Freerouting 2.4.1 on Java 25.
- Blender 4 for renders.
- The simulator's `.venv` for controller tests, `tests/scenarios_round.py` and the scratchpad house replay.

**Spec:** /Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim/docs/superpowers/specs/2026-10-06-r1-low-profile-design.md

**Board plan reused:** /Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim/docs/superpowers/plans/2026-10-04-r1-power-io-board.md. Its ledger and rulings R1–R18 are in `home-robot-r1/.superpowers/sdd/2026-10-04-r1-power-io-board/progress.md`.

## Global Constraints

- **Height and balance:** robot top (LiDAR) ≤ 160 mm. Centre of mass ≥ 15 mm behind the axle; no ballast without asking the user.
- **CAD gates:** build exits 0; clash scan empty; printability all-pass (Prusa MK3S rules: fit 250 × 210 × 210, overhangs, walls, flat base).
- **Jetson:** on 4 insulating standoffs (M2.5, H05) from P_base at its 86 × 58 mm hole pattern. Carrier PCB bottom ≥ (motor top 52.5 and its underside parts 2.23 mm) + 2 mm. Fan top + ≥ 5 mm air gap ≤ lid underside, with vent slots over the fan.
- **Board:** same circuit. board_spec NETS / NETCLASSES / POWER_TRUNK are unchanged.
  - PARTS changes footprints and MPNs for the SMD parts (spec §3): standard 0805 R / C / LED (reflow), 2N7002 SOT-23, SS14 SMA, SMD test pads.
  - SMD is top side only, ≥ 1 mm between SMD bodies, ≥ 2 mm from tall through-hole pads, values and polarity marks on the silkscreen. The user reflows the SMD (paste and stencil), then irons the through-hole parts.
- **Hard robot rules:** no blind reversing; the circle rule for spins; bumper and cliff behaviour unchanged.
- **Don't disturb a running sim:** the live Unreal editor and autonomy service may be in use. Restart them only between steps that need it, and say so.
- **Version control:** commit on branch `power-io-board` in `home-robot-r1` (local git). Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. `home-robot-sim` is not a git repo: back up the files a task changes to `$SP/backup-original/lowprof-L<N>/` first.

Shell variables:

```bash
R1=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-r1
SIM=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim
SP=/private/tmp/claude-501/-Users-liorknafo-Documents-Codex-2026-10-03-i-x20-outputs-home-robot-sim/030356d9-c2ab-4fc2-a131-e83b5b8d9fe0/scratchpad
CADPY=$SP/cadvenv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
```

Work rules for every task:
- No blocking command longer than about 4 minutes; run builds, routing and replays in the background with logs.
- Keep outputs small.
- Never open the large scratch folders f1/, t6/ or t6r/.

---

### Task L1: Jetson and board strip on the base (deck removed)

**Files:** `cad/build_robot.py` (and the CAD outputs it regenerates).

- [ ] Remove P_deck, P_post_1..4, the deck bolts and the base holes that served them.
- [ ] Put the Jetson (official model, as integrated by the Jetson model task) on four H05 standoffs from P_base at its real holes:
  - carrier PCB bottom at z = max(52.5 + 2.23 + 2, rider/pin clearances), about 57;
  - footprint about x −24..55, y ±50;
  - I/O long edge facing +x.
  Choose the exact x so the centre of mass guard holds, and report it.
- [ ] Move the mated USB plugs, the barrel plug and the fan lead to the new ports, keeping the bend room toward the camera tower and the front shell. Use right-angle plugs if a straight plug clashes, and say which ones.
- [ ] Add the P03 regulator envelope (62 × 38 × 14 mm, foam tape) under the Jetson in the front bay, clear of the bumper riders, pins and sweep.
- [ ] Replace the front-bay board with a placeholder strip envelope:
  - PCB 1.6 mm at z 39 on 4 bosses;
  - x −67..−17, y −89..89;
  - module envelopes for Pico, 2 × DRV8874 and GY-521 at plausible spots, each 15 mm above the board.
  The exact outline comes in L4.
- [ ] Move the battery's Tamiya plug and P_harness_saddle_3 out of the strip.
- [ ] Gate: build exits 0, clash scan empty, printability all-pass, COM ≥ 15 mm behind the axle. Report mass, COM, and the Jetson / plug / regulator clearances.
- [ ] Commit.

### Task L2: Lower the upper structure

**Files:** `cad/build_robot.py`, `cad/parameters.json` if heights live there.

- [ ] Introduce one derived height: lid underside = Jetson fan top + 5 mm air gap. Rebuild these from it:
  - lid sectors and lid posts;
  - rear shell and the bumper skin height (today z 18..141);
  - camera tower, so the camera mount drops by the same amount;
  - LiDAR tray and adapter;
  - lid switches (rocker, motor toggle, arm button), each still clearing the battery top (z 64) and the board;
  - vent slots over the fan;
  - all harness routes that went over the deck.
- [ ] Gate: as in L1, plus robot top ≤ 160 mm and every printed part within the MK3S envelope. Report the new LiDAR scan-plane height and camera mount height.
- [ ] Render the review images to the scratchpad and look once.
- [ ] Commit.

### Task L3: Simulator and controller at the new heights

**Files:** `$SIM/Content/RobotData/*` (generated by the sync), `$SIM/tests/*` only where a test hard-codes an old height (each such change explained). The scratchpad house replay harness (house2d) re-sliced at the new LiDAR height with slice_house.py.

- [ ] Back up the files to be changed.
- [ ] Run `.venv/bin/python Scripts/sync_robot_data.py --cad $R1/cad` and report mass and COM.
- [ ] Controller unit suite green.
- [ ] `tests/scenarios_round.py` all PASS; `tests/scenarios_goto_wall.py` all PASS.
- [ ] Re-slice house2d at the new LiDAR plane. Run the whole-house replay for seeds 0 and 1 with the camera, before and after. Report rooms, coverage, contacts and body blocks.
- [ ] Any perception retune for the lower camera must be justified by a failing test or replay. Keep it minimal.

### Task L3b: SMD parts in board_spec and schematic (board Tasks 2/4 data)

- [ ] Change PARTS footprints, symbols and MPNs per spec §3:
  - resistors: `Resistor_SMD:R_0805_2012Metric`;
  - C2: `Capacitor_SMD:C_0805_2012Metric`;
  - LEDs: `LED_SMD:LED_0805_2012Metric`;
  - Q1–Q5: `Package_TO_SOT_SMD:SOT-23` with the KiCad symbol whose pins map S/G/D like the 2N7000 data (verify the 2N7002 symbol's pin numbers against SOT-23 pads);
  - D1: `Diode_SMD:D_SMA` (SS14, check cathode = pin 1);
  - test points: `TestPoint:TestPoint_Pad_D2.0mm`.
  Verify every footprint name exists in KiCad 9.
- [ ] Keep NETS, NC and every connector, socket, fuse holder, relay, terminal and C1 as through-hole.
- [ ] Regenerate the schematic. ERC 0, netlist == NETS, and all Task 2 / Task 4 tests green; update tests that assert the old footprints.
- [ ] Add a part-side field (top SMD) for L5.

### Task L4: Board outline for the strip (board Task 3 redo)

- [ ] board_spec: set BOARD_X0 / BOARD_Y0 / BOARD_W / BOARD_H / Z_PCB_BOTTOM for the strip (frame convention unchanged: u = x − X0, v = Y0 − y), and update REMOVED_CAD_IDS for the L1 placeholders.
- [ ] Regenerate board_outline.json from the L1/L2 CAD with extract_outline.py (real-mesh keep-outs, sweep for moving parts, closing of gaps < 2 mm).
- [ ] Choose the mounting holes.
- [ ] Tests from Task 3 / R10 green.

### Task L5: Dense placement (board Task 5 redo)

- [ ] Rewrite PLACEMENT / MODULES / POWER_ZONES / EDGE_CONNECTORS for the strip:
  - SMD resistors, SOT-23s, LEDs and test pads may sit under the socketed modules;
  - assembly spacing rules (spec §3: ≥ 1 mm SMD–SMD, ≥ 2 mm to tall THT pads, top side only) asserted by a test;
  - connector edges as the spec lists;
  - U5A/U5B on the edge facing the P03 bay;
  - U5C only if it fits (otherwise drop it and say so; the user's P03 is primary).
- [ ] All Task 5 tests green: module alignment, courtyards, trunks with R13 thermals, net classes.

### Task L6: Routing (board Task 6 redo)

- [ ] route.py with the fix-1a tests (R15 return paths, R16 motor clearance, R17 via-to-pad / hole keep-out / clean-only writes / placement parity / no GND tracks).
- [ ] Pre-routes as needed.
- [ ] DRC 0 / 0 unconnected.
- [ ] If 2 layers cannot route the strip within these rules after a real effort, stop and report with numbers: a 4-layer board needs the user's approval (cost).

### Task L7: Power-path check (board Task 7, R4-restated)

- [ ] check_power on the routed board:
  - Power-class tracks ≥ 3.0 mm;
  - motor tracks ≥ IPC width for 2.1 A;
  - trunk erosion check for VBAT_SW / VMOT / 5V / GND with R13 reliefs.
- [ ] Amend the brief's "≥ 3 mm for every CURRENT_A net": the PowerTap taps are 0.8 mm by R4.

### Task L8: Fabrication package (board Task 8)

- [ ] Gerbers (including F.Paste for the stencil), drill, BOM, 1:1 PDF, STEP with the plug-in modules and the missing 3D models (fuse holders, VH headers) as envelopes.
- [ ] ORDERING.md notes:
  - P03 foam tape and its measurement request;
  - FLR holder on a 1.6 mm board;
  - up-angled micro-USB;
  - no D24V50F5 (U5C) on this board: it was dropped in L5 (no room on the strip); the P03 is the only 5 V regulator;
  - order a solder-paste stencil (top);
  - assembly order: reflow SMD, then THT, then modules.

### Task L9: Real board into the robot CAD (board Task 9)

- [ ] Board STEP plus module parts (Pico, DRV8874 ×2 from the official models; GY-521; P03 envelope) replace the L1 placeholders.
- [ ] Bosses with M3 inserts at the outline holes.
- [ ] Clash scan, including the bumper sweep near the IMU if it still applies.
- [ ] Printability, mass and COM.

### Task L10: Harness and BOM (board Task 10)

- [ ] Short harness from the strip's connectors to the battery, rocker, Jetson, cliffs, bumpers, arm and toggle.
- [ ] update_bom.py.
- [ ] Cable list regenerated; wire clash check clean.

### Task L11: Simulator sync, bring-up guide, live mapping benchmark

- [ ] Sync the robot data and refresh the robot model in Unreal: restart the editor only now, and tell the user.
- [ ] BRINGUP.md.
- [ ] Final spec checks for both specs.
- [ ] Run the Unreal full mapping benchmark (`tests/full_mapping_run.py` + `tests/score_full_mapping.py`); it must pass.
- [ ] Re-render the review images and the 3D viewer artifact.
