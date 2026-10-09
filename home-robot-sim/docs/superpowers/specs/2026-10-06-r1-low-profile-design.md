# R1 low-profile redesign: design spec

Date: 2026-10-06. Status: direction approved by the user ("Yes, full redesign"). Robot: home-robot-r1, the round robot that turns about its centre.

## 1. Goal

Make the robot about 42 mm lower, from 198 mm to about 156 mm at the LiDAR top. To do that, remove the electronics deck and put the Jetson, the main board and the battery all on the base level. Everything else stays as it is: mapping behaviour, the bumper and cliff sensing, the drive, and the circuit of the power & I/O board.

## 2. Decisions

| Topic | Decision | Why |
|---|---|---|
| Deck | Removed, with its 4 posts (P_post_1..4) and deck bolts | It is what holds the Jetson high |
| Jetson | Official NVIDIA B01 model, on 4 insulating standoffs from P_base at its real 86 × 58 mm M2.5 hole pattern. Carrier PCB bottom at z ≈ 57. Footprint spans robot x ≈ −24..55 (79 mm, over the motors), y ±50. I/O long edge faces forward (+x). | The base has no room for it flat on the floor. The motor tops (z 52.5) and the bumper rail pins (z 53) set the minimum height. Spanning over the axle keeps the centre of mass behind it. |
| Main board | Same circuit (board_spec NETS / PARTS unchanged), new outline: a strip of about 48 × 178 mm between the battery tray and the motors (robot x ≈ −67..−17, y ≈ −89..89), long axis along y, bottom at z 39 (6 mm bosses). Low parts sit under the socketed modules. | That is the only base area left (spike: 51 × 81 mm pockets each side of the battery's Tamiya plug, joined once the plug and harness saddle 3 move) |
| 5 V regulator (P03) | Off-board, on foam tape under the Jetson in the front bay (z 33..56 space), leads to U5A/U5B on the board edge facing it. U5C (D24V50F5 socket) stays on the board only if it fits. | 62 × 38 mm does not fit the strip |
| Battery | Unchanged position (tray x −121..−71). Its Tamiya plug moves out of the strip, to an adapter lead into J1 on the strip's rear edge. | The plug alone blocked the middle of the strip |
| Height chain | Jetson PCB z 57 → heatsink top 85.3 → 40 mm fan top 95.3 → lid underside ≥ 100 (air gap) → lid top ≈ 103 (was 146). | |
| Upper structure | Lid sectors, rear shell, lid posts, the bumper skin (z 18..141 today), the camera tower and the LiDAR tray all come down by the same ≈ 43 mm. The lid switches (master rocker 28 mm, motor toggle, arm button) still hang inside the lid above the battery top (z 64). | |
| Sensors | LiDAR scan plane 0.19 → ≈ 0.147 m. Camera mount height 0.150 → ≈ 0.107 m. Cliff sensors and bumper unchanged. | They follow the lid |
| Balance | Centre of mass must stay ≥ 15 mm behind the axle (existing guard). Estimate ≈ −17 mm. No ballast without asking the user. | |

## 3. SMD parts, reflowed by the user (user decisions 2026-10-06)

The user assembles the board himself and reflows the SMD parts with a reflow station, using paste and a stencil.

**Surface-mount (top side only):**
- Resistors, C2 and the LEDs: 0805, standard KiCad footprints (not HandSolder; IPC nominal pads align and resist tombstoning under reflow).
- MOSFETs: 2N7002 in SOT-23, the SMD equivalent of the 2N7000.
- Flyback diode: SS14 in SMA, instead of the 1N5819.
- Test points: SMD pads ≥ 2 mm.
- Nothing finer than SOT-23.
- No SMD on the bottom side, so the board can go on a hot plate.

**Still through-hole, soldered by iron after reflow:**
- Every connector.
- The module sockets.
- The fuse holders.
- The relay.
- The regulator screw terminals.
- C1 (470 µF electrolytic).

**Assembly rules:**
- ≥ 1.0 mm between SMD bodies, for rework.
- ≥ 2.0 mm between an SMD pad and the pads of tall through-hole parts (relay, fuse holders, C1, sockets), so hot air is not blocked.
- Thermal reliefs (R13) on SMD pads that join a copper zone.
- Values and polarity marks (LED cathode, diode band, SOT-23 pin 1) on the silkscreen.
- The fab package adds the F.Paste layer and asks for a stencil.
- BRINGUP.md gives the order: paste, place, reflow the SMD; inspect; then iron the through-hole parts; then the modules.

SMD parts may sit under the socketed modules.

## 4. Board consequences

The power & I/O board plan (docs/superpowers/plans/2026-10-04-r1-power-io-board.md) keeps everything electrical:
- Tasks 1, 2 and 4: tools, nets and parts, schematic and ERC.
- Rulings R1–R18: net classes, thermal reliefs, return-path tests, via-to-pad, hole keep-outs.

These must be redone for the new outline:
- Task 3: outline from the new CAD.
- Task 5: dense placement, with low parts under the Pico and the drivers.
- Task 6: routing.
- Task 7: power check.
- Tasks 8–11: fab files, CAD integration, harness and BOM, simulator sync and bring-up.

Connector edges follow the cables:
- J1 battery: rear edge.
- J4 / J5 motors: strip ends near each motor header.
- J3 Jetson: front edge, the lead runs over the motors.
- J2 rocker and J13 toggle loop: rear edge.
- J6–J9 cliffs: front pair along the side ends, rear pair rear edge.
- J10 / J11 bumpers and J12 arm: as short as possible.

## 5. Simulator and controller

The robot data sync (Scripts/sync_robot_data.py) carries the new heights, mass, centre of mass and meshes into Content/RobotData. The controller reads geometry from robot-parameters.json, so no constants change. Camera perception (mono / floor parallax) and LiDAR tests must pass at the new heights.

Acceptance is the existing evidence, re-run:
- 2D scenarios: all pass, including the no-blind-reverse audit.
- Whole-house replay: rooms entered, at least 90 % mapped, no body blocks.
- Unreal full mapping benchmark: all rooms entered, at least 90 % each, robot reports complete, within the 1500 s budget.

## 6. Acceptance

- CAD: build exits 0. The clash scan is empty, printability is all-pass, the mass and centre-of-mass checks pass (COM ≥ 15 mm behind the axle), and the robot top is ≤ 160 mm.
- Board: outline inside the strip with ≥ 1.5 mm to real geometry. DRC 0 / 0 unconnected, R4/R13 trunk check, R15 return paths, R17 via and hole rules.
- Simulator: synced, controller unit tests green, scenarios all pass, house replay no worse than before, and the Unreal mapping benchmark passes.

## 7. Risks

- **Board density on 2 layers:** reduced by the SMD change (§3), which frees most of the resistor and transistor area.
- **Previous estimate, before SMD:** about 6,600 mm² of footprints on about 8,500 mm², autorouted by Freerouting, which was already fragile on the bigger board. Mitigations: parts under the modules, more scripted pre-routes, and a 4-layer fallback that needs the user's approval because it changes the cost.
- **Balance margin:** about 2 mm by estimate. The CAD decides. If it fails, the battery or board shifts, or I ask the user about ballast.
- **Camera perception at the lower height:** the mono / floor-parallax tuning may need retuning.
- **Jetson cooling under a lower lid:** keep vent slots over the fan and a ≥ 5 mm air gap.
- **Plug room:** about 45 mm between the Jetson's port edge and the camera tower and bumper shell for USB and Ethernet plugs. Use right-angle plugs if the CAD shows a clash.
