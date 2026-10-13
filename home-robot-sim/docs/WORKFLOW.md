# Workflow: one task = one Linear issue, one branch, one T3 thread (from 2026-10-11)

The controller thread (T3 thread `8164f576-11ab-40d0-bc58-365bcb1118db`, project home-robot-sim) owns the plan, the Linear project
"Home Robot R1" (team LIO), the merges and the GitHub sync. Task threads do one task each and report back to it.

## Locations

```bash
R1=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-r1    # git; integration branch power-io-board
SIM=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim  # git since 2026-10-11; integration branch main
PUB=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot      # public GitHub snapshot (tools/sync.sh); controller only
LEDGER=$R1/.superpowers/sdd/2026-10-08-r1-bay-board   # git-ignored, so NOT in worktrees: always use this absolute path
SIMPY=$SIM/.venv/bin/python                            # sim Python (cv2, numpy); homebrew python has no cv2
CADPY=/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/.venv-cad/bin/python  # CadQuery 2.8.0 + trimesh 5.1.1, permanent (rebuilt 2026-10-11 after the /tmp venv was wiped)
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3   # KiCad 9.0.9 pcbnew
KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
JAVA=/opt/homebrew/opt/openjdk@25/bin/java   # Freerouting jar: $R1/pcb/tools/freerouting.jar (git-ignored: worktrees use this absolute path)
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender
```

## Branches and worktrees

- Work only in your own worktree and branch. The main checkouts of R1 (`power-io-board`) and SIM (`main`) stay clean; only the controller
  merges into them.
- A task that touches both repos uses the same branch name in both. The thread's bound worktree is in one repo; create the other with
  `git -C <repo> worktree add <path> -b <branch> <integration-branch>` and say where in your notes.
- Sim sync from a worktree: `$SIMPY <sim-worktree>/Scripts/sync_robot_data.py --cad <r1-worktree>/cad` (the default `--cad` is the
  sibling main R1 folder, which is the wrong one for a branch).
- Commit after each green step. Stage only the files you changed (never `git add -A`). End messages with
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Put the Linear issue id (LIO-nn) in the subject when you have it.
  Do not push; the controller syncs GitHub.

## The live simulator is one shared resource

Unreal (the user's simulator), the autonomy service and the dashboard run only from the main `$SIM` folder, for one task at a time.

1. Take the lock: `mkdir $SIM/.sim-lock` (it fails if someone holds it: wait and retry every few minutes; never break someone else's
   lock). Write `owner` in it: thread, task, what you run (say `benchmark` for a timed Unreal benchmark), start time.
2. While you hold it you may `git -C $SIM switch --detach <your-branch>` to run Unreal with your changes (plain `switch` fails while the
   branch is checked out in your worktree; the main folder must be clean before you switch), and must `git -C $SIM switch main` again
   before you release. Then rebuild Unreal for `main` if your branch changed C++, so the running simulator matches `main`.
3. Unreal etiquette: back up `~/Library/Logs/HomeRobotSim/` before every relaunch (UE log rotation deleted logs once); quit gracefully
   with UDP `{"quit":true}` to 8767 and wait; never SIGKILL the user's editor; relaunch with `Scripts/launch.command house polished`;
   start the autonomy service with `$SIMPY` from `controller/` (port 8771). The user opens the dashboard from their phone and laptop:
   add `--host 0.0.0.0` (see "Opening the dashboard from a phone or laptop" below).
4. Release: leave the simulator running on `main` with the service up (the user expects it running), then `rm -rf $SIM/.sim-lock`.
5. While the lock owner says `benchmark`, do not start route.py / Freerouting or a CAD build (they skew the timing), and keep other
   sim runs light (JOBS=2, at most about 4 processes across all threads). The benchmark owner notes the machine load in its report. One Freerouting JVM
   at a time on this Mac (`-Xmx1g`, one worker).

## Opening the dashboard from a phone or laptop

The robot dashboard normally answers only on this Mac (`http://127.0.0.1:8771/autonomy.html`). To open it from another device on the
home Wi-Fi, start the service with `--host 0.0.0.0`:

```bash
Scripts/start-autonomy.command --host 0.0.0.0        # or: cd controller && $SIMPY autonomy_service.py --host 0.0.0.0
```

- On start it prints the addresses that work, for example `http://Liors-MacBook-Pro.local:8771/autonomy.html` and
  `http://192.168.1.201:8771/autonomy.html`. Type either one into the phone's or laptop's browser. The `.local` name keeps working
  when the router gives the Mac a new IP address; the IP address is the fallback if a device cannot find `.local` names.
- If the macOS firewall is on, macOS asks once whether Python may "accept incoming network connections". Click Allow. If you clicked
  Deny, change it in System Settings > Network > Firewall > Options.
- There is no password. Anyone on the home Wi-Fi who opens the page can start Mapping, send the robot somewhere or stop it.
- Only the dashboard is shared: autonomy.html, the house preview and LiDAR review pages it links, the files those pages load and the
  `/api` routes. Every other file in the sim folder answers 404 (not found), on this Mac too while the flag is on (restart without
  the flag to see the other review pages). Without `--host` nothing changes.
- The robot model page (the R1 design review, with the 3D robot and the STL/STEP downloads) is at `/robot/design-review.html` on the same
  address, in both modes. It is read from the `home-robot-r1` folder next to the sim folder; from anywhere else add `--r1-dir <folder>`.
  Only the files that page loads or links are served from the R1 folder.
- Double-clicking `Scripts/start-autonomy.command` in Finder starts it for this Mac only; the flag needs Terminal.

## Notes, reports, review

- Your log and report go in `$LEDGER/task-<id>-notes.md` and `task-<id>-report.md`. Read `$LEDGER/progress.md`,
  `global-constraints.md` and `final-review-carry.md` for history and rulings, but do not edit `progress.md` (the controller's ledger).
- TDD where tests apply. Before you call the task done: every gate it touches is green, the report is written in plain language with the
  numbers, and an independent review passes (the `astra` agent: work continues until it answers ACCEPT).
- If your task touches the controller, the robot geometry or the sim sync, the scenario gate is **both** suites:
  `JOBS=4 $SIMPY tests/scenarios_round.py` and `R1_SENSOR_ERRORS=off JOBS=4 $SIMPY tests/scenarios_round.py` (rc 0 each), plus
  `tests/scenarios_goto_wall.py`. List both in your report (task F's merge missed the errors-off suite).
- When done or blocked, tell the controller: `t3_thread_send` to thread `8164f576-11ab-40d0-bc58-365bcb1118db` with the branch, last
  commit, test results and anything the user must decide.

## Sub-agents

Pick the cheapest model that fits each sub-agent job (user rule):

- **Claude Haiku 5.5:** mechanical, fully specified, easy-to-check jobs: searches, pulling numbers out of logs, running a given test
  command and reporting the counts, bookkeeping.
- **Claude Sonnet 5.5:** well-scoped standard work with a clear brief: a step from a clear plan, writing tests, small refactors, a
  focused investigation, summarizing long reports, first-pass checks.
- **Your inherited model:** design decisions, hard debugging, safety rules, and anything where a mistake is costly or hard to spot.
  astra keeps its own model.
- How: native Agent tool `model: "haiku"` / `"sonnet"`; T3 `delegate_task` `target: {providerInstanceId: "claudeAgent", model:
  "claude-haiku-5-5"}` / `"claude-sonnet-5-5"`. Check the results; if a smaller model struggles, re-run one tier up.

## User rules (binding)

- Act, don't ask: fix diagnosed problems directly. Ask only for downloads (name, source, size), purchases, deletes and irreversible
  actions. Never order or pay for anything.
- Nothing is built or ordered yet: printed parts and the board may change freely.
- Printed parts must print on the user's Prusa MK3S (250 × 210 × 210 mm bed, sensible overhangs, walls, a flat base).
- Use NVIDIA's official Jetson Nano B01 model in CAD, renders, viewer and sim.
- Write for the user in plain language: short sentences, numbers with units, no unexplained jargon.
