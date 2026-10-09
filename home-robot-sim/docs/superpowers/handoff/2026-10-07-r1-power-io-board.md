# Handoff: R1 low-profile robot and its power & I/O board (2026-10-07)

This document lets another agent take over. Read it whole, then the ledger it points to. Trust the ledger and `git log` over any memory of the previous session.

## 0. TL;DR

- **The robot:** home-robot-r1, a round robot that turns about its centre. Its CAD is lowered from 198 to 155 mm. The Jetson Nano now sits on standoffs over the motors, and the main board is a 50 × 178 mm "strip" between the battery and the motors. Tasks L1–L5 of the low-profile plan are **done and reviewed**.
- **The blocker is routing the board (L6).** On 2 layers, the strip with the L5 placement could not be routed within the rules. Two lines of work were running when the user stopped the session:
  1. **L5b, a 2-layer placement rework.** This was the user's choice: keep 2 layers, $8.30 for 5 boards. It is **close**. The best run had 0 unconnected and DRC 0, failing only two quality limits. The WIP is committed as `751f0fa` on `power-io-board`.
  2. **S4L, a spike: split into a 4-layer logic board ≤ 50 × 100 mm plus a 2-layer power board.** The user asked for this because JLCPCB sells 4-layer boards ≤ 102 × 102 mm at $8 for 5, and 2-layer ≤ 100 × 100 mm at $4. Only fit studies exist; nothing is committed. It is in worktree `$SP/r1-4L`, branch `spike-4L-split`.
- **Open decision for the user:** finish L5b (2-layer, one board) or go two-board (4L + 2L). No agent is running now. No Freerouting process is running.

## 1. Locations

```bash
R1=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-r1      # local git, branch power-io-board
SIM=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim    # NOT git; Unreal 5.8 sim + Python autonomy service
SP=/private/tmp/claude-501/-Users-liorknafo-Documents-Codex-2026-10-03-i-x20-outputs-home-robot-sim/030356d9-c2ab-4fc2-a131-e83b5b8d9fe0/scratchpad   # old session scratch (tmp: may vanish on reboot)
CADPY=$SP/cadvenv/bin/python            # CadQuery 2.8 + trimesh venv (pure pcb tests, CAD). Recreate if $SP is gone.
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3   # KiCad 9.0.9 pcbnew
KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
JAVA=/opt/homebrew/opt/openjdk@25/bin/java   # Freerouting 2.4.1 needs Java 25; jar at $R1/pcb/tools/freerouting.jar (git-ignored)
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender   # Blender 4 (viewer GLB composition, renders)
```

**Key documents:**

| What | Path |
|---|---|
| Low-profile design spec (binding) | `$SIM/docs/superpowers/specs/2026-10-06-r1-low-profile-design.md` |
| Low-profile plan (tasks L1–L11) | `$SIM/docs/superpowers/plans/2026-10-06-r1-low-profile.md` |
| Original board spec and plan (rulings R1–R18) | `$SIM/docs/superpowers/specs/2026-10-04-r1-power-io-board-design.md`, `.../plans/2026-10-04-r1-power-io-board.md` |
| **SDD ledger, low-profile (read first)** | `$R1/.superpowers/sdd/2026-10-06-r1-low-profile/progress.md` (rulings LP1–LP13, every task's status) |
| SDD ledger, original board plan | `$R1/.superpowers/sdd/2026-10-04-r1-power-io-board/progress.md` (rulings R1–R18) |
| Task briefs, reports, reviews | same folder: `task-L*-brief.md`, `task-L*-report.md`, `task-L5-review.md`, `review-*.diff` |
| Routing notes, binding rules restated | `task-L6-notes.md` (R4, R13, R15, R16, R17, the anti-stall work rules) |
| L6 / L5b report and router log | `task-L6-report.md` (sections 1–9 = the L6 BLOCKED analysis; "## L5b + L6 retry" = **stale interim**, see §3), `task-L6-plan-notes.md` (the L5b agent's live log, most current) |
| L5b brief | `task-L5b-brief.md` |
| S4L spike brief and notes | `task-S4L-brief.md`, `task-S4L-plan-notes.md` |
| Scratch saved for the handoff | `handoff-scratch/{viewer,show,l5b,l6,s4l}` (copies of the small scratch scripts, JSON and plots) |

The `.superpowers/` folder is git-ignored. Its files are on disk but not in git.

## 2. Done (reviewed)

**Low-profile plan:**
- **L1:** Jetson and strip on the base, deck removed.
- **L2:** upper structure lowered. Lid top at z 103.5, robot top 155.15 mm, fan skirt, bumper wall 3.8 mm.
- **L3 (sim / controller):** sim data synced from `cad/mass-properties.json`; overhang stall marks; unit tests 166 OK + 1 expected failure.
- **L3b:** SMD parts for reflow.
- **L4:** strip outline from the CAD.
- **L5:** placement, plus fix round 1. The fix round put K1 2.0 mm from the Jetson-carrier height band, added silk values and polarity marks, and deleted U5C.

Commits on `power-io-board` (newest first):
- `751f0fa` L5b WIP (see §3)
- `ed52c34`, `603c51d` L6 WIP: route.py for the strip, pre-routes, SMD GND vias
- `b2ecd9e` L5 fix 1
- `39d9614` L5
- `b61db36` L4
- `ba6172e` / `89432a2` L2
- `15509b6` L1 fix
- ...

**Earlier work, still in force:**
- Relocalization fix.
- Near-wall Go-to (about 2 cm gap).
- Official 3D models: NVIDIA's Jetson Nano B01 model, plus the Pico and Pololu models in `$R1/reference-models/`.
- Interactive 3D viewer artifact: https://claude.ai/artifact/3iX75ZyJSkk9HHurZMSUMp, version 8, showing the L5-fix placed board.

## 3. In flight: L5b, the 2-layer placement rework (user's chosen path)

**State:**
- Commit `751f0fa`: "pcb: L5b WIP - 2-layer placement rework for routability (unrouted)".
  - The drivers sit after the Pico with their rows along v, and both motor pairs cross the carrier band once.
  - VBAT_SW stays in the rear-right corner.
  - Scripted B.Cu hops (`HOP_VIAS`) carry every barrier crossing.
  - The Pico GPIO map is in one place: `PICO_GPIO` in board_spec.py.
  - route.py runs with no critical stage, no hop windows, R15 keeps for the signal returns, `-Xmx1g` and one worker.
- **Uncommitted:** `parts-and-connections.json` (R1 root) holds the new Pico pin table that matches `PICO_GPIO`. Review it, then commit it with the next L5b commit.
- **Tests at `751f0fa` (re-run 2026-10-07 by the controller):** pure 67 OK; `ktest_pcb` 30 OK. ktest_route is red because the board is not routed yet.

**Best router results so far** (from `task-L6-plan-notes.md`; scratch boards v10–v12 under `$SP/L5b/`; placement scripts copied to `handoff-scratch/l5b/place_v*.py`):
- **run8, grid 0 (v11): 0 unconnected, DRC 0, trunks OK, GND pads OK.** It fails only quality: VSENSE took a 74 mm router B.Cu run (the limit is 40), which pushes J4.3–U3P.2 to 1.62× (the limit is 1.2).
- **v12 scripts VSENSE.** run9 grid 0: 2 open (R_CS and SCL, Pico side), and SLEEP took a 55 mm B.Cu run. run9 grid 1: 2 open (3V3), longest B.Cu 41.0 mm, R15 1.00×.
- **The remaining knot is the Pico's +v exit.** Nets leave the inner channel between the socket rows in an order that forces about 20 crossings.
  - The agent's next idea (v13) was a GPIO remap to match the exit order: `[L_CS, FAULT, R_CS, SLEEP | U3L 3V3, L_IN2, L_IN1 | U4L 3V3, R_IN2, R_IN1 | XSHUT4, SCL, SDA, 3V3, ENC_RB, ENC_RA, MOTOR_ON_N]`.
  - Caveat: U1A has 16 GPIOs for 17 nets that want the rear row.
- **`task-L6-report.md` "## L5b + L6 retry" is stale.** It says "BLOCKED (disk), nothing committed". The disk was freed afterwards and `751f0fa` was committed. Replace that section with the final one.

**Next steps for L5b:**
1. Check `751f0fa` matches v12. Compare `pcb/board_spec.py` against `handoff-scratch/l5b/place_v12.py` and `board_spec_v12_repo.py`.
2. Route `751f0fa` with route.py, one grid at a time.
3. Try the v13 GPIO remap and/or more grids and wiring orders.
4. Acceptance (task-L6-notes.md):
   - DRC 0 / 0 unconnected;
   - all ktest_route tests green: R15 ≤ 1.05× for the power loops and ≤ 1.2× for the signal returns; longest router B.Cu run ≤ 40 mm; R16 Motor clearance 1.0 mm; R17 vias ≥ 0.5 mm and washer keep-outs; zero GND tracks plus the LP12 SMD GND vias;
   - ktest_pcb and the pure tests stay green.
5. Then report, review (SDD), and continue with L7.
6. If it cannot pass, report the numbers. The user decides between 4 layers and the split. **Never switch layers yourself.**

## 4. In flight: S4L spike, 4-layer logic board plus 2-layer power board (user's "try 4 layers < 102 × 102")

**JLCPCB instant quote** (2026-10-07, default options, 1.6 mm, 5 pcs):

| Size | 2 layers | 4 layers |
|---|---|---|
| ≤ 100 × 100 mm (offer applies at 50 × 102, not at 50 × 103) | $4 | $8 |
| 50 × 178 mm | $8.30 | $30.50 |

Shipping is about $31 by DHL either way. One 50 × 102 board cannot hold every part: about 4,400 mm² of effective footprint against about 5,100 mm².

**Spike state:**
- Worktree `$SP/r1-4L`, branch `spike-4L-split` from `ed52c34`. **No commits, no code changes.**
- Fit studies are in `$SP/S4L/` (copied to `handoff-scratch/s4l/`: `fit.py`, `lp1.py`, `lp2.py`, `autoplace.py`, plots).
- From `task-S4L-plan-notes.md`:
  - **Split:** logic board at the LEFT end (strip v 0–100, 4 layers), power board at the RIGHT end (v ≈ 106–178, 2 layers, all-THT, D1 → 1N5819).
  - **Link:** Micro-Fit 3.0, 2 × 4 pins (VMOT × 2, GND × 2, 5V, COIL_P, VSENSE, LED1_A). R11 and R21 move to the power board as THT.
  - **lp2:** all THT parts fit, but only by using the Pico band exemption at 0.6 mm and putting 2 new inserts in the band. The topology is fatal: the 5 A VMOT would have to cross the L rows.
  - **lp3 plan:** link in the front-left block; VMOT under the IMU and along the band; J4 and J5 along the front edge; J14 reduced to a 1 × 4 pin header. lp3 was being built when the spike was stopped.
  - **Verdict so far:** the logic board fits only barely. Routability is untested.
- If the user picks the split, finish the spike per `task-S4L-brief.md`. If the user drops it, remove it: `git -C $R1 worktree remove $SP/r1-4L` and `git -C $R1 branch -D spike-4L-split`. Neither is user data.

## 5. Remaining plan after routing (low-profile plan L7–L11)

- **L7: power-path check.** The brief's "≥ 3 mm for every CURRENT_A net" conflicts with R4, where PowerTap taps are 0.8 mm. Amend that line in the dispatch.
- **L8: fab package.**
  - Gerbers including F.Paste, drill, BOM, 1:1 PDF, and a STEP with the modules plus envelopes for the parts that have no 3D model (FLR fuse holders, VH headers).
  - ORDERING.md: P03 foam tape and its measurement request, FLR holder on a 1.6 mm board, up-angled micro-USB, order a top stencil, assembly order (reflow SMD, then THT, then modules), keep via tenting on.
  - L8 also owns the deferred L5 minors: silk text 0.8 → 1.0 mm if every label still places, and fix the board_spec.py exemption comment (carrier underside parts are 14.17 mm above the board top, not 15.0).
  - The plan already says "no D24V50F5 (U5C) on this board".
- **L9: real board into the robot CAD** (`$R1/cad/build_robot.py`).
  - Clash scan with the real meshes, including the Pico exemption and the GY-521 over the bumper-pin cutout.
  - Lid STLs pre-flipped, the LD14 lead aperture airflow, and design-review.html regenerated.
  - Any new heat-set inserts, if the board split is chosen.
- **L10: harness and BOM.** Cable lengths follow the connector edges and the GPIO swaps; parts-and-connections.json; drop the D24V50F5 from the BOM; strip_drv_* BOM code E06 is wrong.
- **L11:**
  - Sim sync, then restart the autonomy service and the Unreal refresh. The **running autonomy service still has pre-L3 code**. Start it with `$SIM/.venv/bin/python`; homebrew python has no cv2.
  - Write BRINGUP.md:
    - lid-on tegrastats test and the 0.5 mm skirt feeler check;
    - plug the right motor lead before fitting the Jetson;
    - reflow, then THT, then modules.
  - **Re-record the 5 camera fixture sets** in Unreal at camera_mount_m: bedroom-edges, floor-rejection, floor_parallax, slipper-approach, slipper-side-approach.
  - RobotPawn inertia; LiDAR collision box (160.3 mm, box vs cylinder).
  - Run the Unreal mapping benchmark: `tests/full_mapping_run.py` + `score_full_mapping.py`. All 5 rooms ≥ 90 %, robot reports complete, ≤ 1500 s.
  - Update the final viewer.
- **Open on the user's side:** measure the P03 regulator, the USB plugs and the Wi-Fi dongle; confirm a B01 kit with the J48 jumper.

## 6. User rules and preferences (binding)

- **Act, don't ask.** Fix diagnosed problems directly. Ask only for downloads (name, source, size), purchases, deletes and irreversible actions. Never order or pay for anything.
- **Never delete the user's data.**
  - `$SIM/Saved/Stereo` (31 GB) is theirs.
  - Agents must not permanently delete user files. The user runs `rm` themselves.
  - Your own scratch files and regenerable build caches are fine to remove.
- **Always use NVIDIA's official Jetson Nano B01 model** in the CAD, renders, viewer and sim sync, never a block (memory: project-real-jetson-model).
- **Every printed part must print on the user's Prusa MK3S:** 250 × 210 × 210 build volume, overhangs, walls, a flat base.
- **The user assembles the board by hand:**
  - SMD with paste, stencil and a reflow station, top side only;
  - then irons the THT parts;
  - spacing: ≥ 1 mm SMD to SMD, ≥ 2 mm from THT pads;
  - silk values and polarity marks.
- **Robot behaviour rules:**
  - Reverse only into space the robot has already seen free; no blind reversing.
  - Circle rule for spins.
  - Bumper and cliff behaviour unchanged.
- **Don't press dashboard buttons during trials.** Don't disturb a running sim: the **Unreal editor is running**, so restart it only between steps that need it, and say so.
- **Version control:**
  - Commit on `power-io-board` in `$R1`.
  - Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
  - `$SIM` is not git. Back up files before changing them, in `$SP/backup-original/lowprof-L<N>/`, or in a durable folder if `$SP` is gone.

## 7. Machine limits (learned the hard way)

- **One Freerouting JVM at a time, `-Xmx1g`.** Five parallel JVMs plus the Unreal editor pushed macOS swap from 14 to 18 GB. Swap shares the disk, so free space fell under 1 GB.
- Check `df -h /System/Volumes/Data` before each router run. It shows about 205 GB free as of this handoff.
- Run long jobs in the background with logs. Nothing should block for more than about 4 minutes.
- Agents doing long design thinking stalled on the stream watchdog before (the original Task 6 had 5 stalls). Write plans to a notes file first and work in small steps.
- Never open the old large scratch folders `$SP/f1`, `$SP/t6`, `$SP/t6r`.

## 8. 3D viewer (artifact 3iX75ZyJSkk9HHurZMSUMp)

The viewer loads three base64 `.glb.txt` files, because the artifact host does not serve `.glb`:

| File | Content | Built by |
|---|---|---|
| `r1-robot.glb.txt` | the robot CAD; strip plate, CAD module placeholders and the simplified Jetson hidden | `viewer/compose.py` (Blender, BUDGET 115000) |
| `jetson.glb.txt` | NVIDIA's full B01 STEP | `show/jetson_full.py` + `viewer/jetson_glb.py`; placed at rot +90° z, origin (25, −46, 57) |
| `r1-board.glb.txt` | KiCad's own 3D export of the board, plus the modules on their real socket pads | see below |

How `r1-board.glb.txt` is made:
1. Export from KiCad: `kicad-cli pcb export glb --subst-models --include-tracks --include-pads --include-silkscreen --include-soldermask --user-origin 0x0mm`.
2. Note that the GLB's z 0 is the **board bottom**. Offset it by (BOARD_X0, BOARD_Y0, Z_PCB_BOTTOM).
3. Place the modules with `show/modules_l5.py`, from the socket pads in `pads5.json`. The script takes an optional board_spec snapshot root as argv[3].
4. Compose with `viewer/board_glb.py`.

All sources are copied in `handoff-scratch/viewer` and `handoff-scratch/show`. The page is `handoff-scratch/viewer/r1-robot-model.html`; its hover names come from BOARD_NAMES, built from PARTS / RES.

To update the published page from a new session, read the artifact first (`action: read` on the URL), then publish with `url`. Re-run the viewer after routing (tracks) and after L9.

## 9. Suggested first moves for the next agent

1. Read the low-profile ledger `progress.md` (from the "Task L5" lines down), this file, then `task-L6-plan-notes.md` (its last 40 lines).
2. Check the state:
   - `git -C $R1 log --oneline -5` should show `751f0fa`.
   - `git -C $R1 status` should show only `parts-and-connections.json` modified and the untracked `reference-models/pico-r3/__MACOSX/`.
   - `df -h /System/Volumes/Data`.
3. Ask the user which path to take: L5b (2-layer) or the S4L split. Then continue with the SDD process: brief, implementer, review, ledger.
