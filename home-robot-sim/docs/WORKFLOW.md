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
CADPY=/private/tmp/claude-501/-Users-liorknafo-Documents-Codex-2026-10-03-i-x20-outputs-home-robot-sim/030356d9-c2ab-4fc2-a131-e83b5b8d9fe0/scratchpad/cadvenv/bin/python  # CadQuery 2.8 + trimesh (in /tmp: if gone, recreate a venv with cadquery==2.8 trimesh and note where)
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
   start the autonomy service with `$SIMPY` from `controller/` (port 8771).
4. Release: leave the simulator running on `main` with the service up (the user expects it running), then `rm -rf $SIM/.sim-lock`.
5. While the lock owner says `benchmark`, do not start route.py / Freerouting or a CAD build (they skew the timing). One Freerouting JVM
   at a time on this Mac (`-Xmx1g`, one worker).

## Notes, reports, review

- Your log and report go in `$LEDGER/task-<id>-notes.md` and `task-<id>-report.md`. Read `$LEDGER/progress.md`,
  `global-constraints.md` and `final-review-carry.md` for history and rulings, but do not edit `progress.md` (the controller's ledger).
- TDD where tests apply. Before you call the task done: every gate it touches is green, the report is written in plain language with the
  numbers, and an independent review passes (the `astra` agent: work continues until it answers ACCEPT).
- When done or blocked, tell the controller: `t3_thread_send` to thread `8164f576-11ab-40d0-bc58-365bcb1118db` with the branch, last
  commit, test results and anything the user must decide.

## User rules (binding)

- Act, don't ask: fix diagnosed problems directly. Ask only for downloads (name, source, size), purchases, deletes and irreversible
  actions. Never order or pay for anything.
- Nothing is built or ordered yet: printed parts and the board may change freely.
- Printed parts must print on the user's Prusa MK3S (250 × 210 × 210 mm bed, sensible overhangs, walls, a flat base).
- Use NVIDIA's official Jetson Nano B01 model in CAD, renders, viewer and sim.
- Write for the user in plain language: short sentences, numbers with units, no unexplained jargon.
