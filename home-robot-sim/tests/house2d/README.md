# Whole-house 2D replay harness

Closed-loop whole-house mapping with the real robot code (`controller/autonomy`) in `tests/sim2d.py`. It runs about 2-4x
faster than real time with no Unreal. The house is 2D slices of the Blender house meshes. Moved here from /tmp in bay B13.

| File | What it is |
|---|---|
| `run_house.py` | One mapping run: `run_house.py LABEL [seconds=1600] [seed=0] [cam] [ovcam]` |
| `house_world.py` | Loads the slices into a sim2d `World` and emulates the camera's low-obstacle detections (`cam`) |
| `house_slices_lp1275.npz` | **Default** (task L): LiDAR plane 0.1275 m, an overhang slice up to the robot top 0.13515 m. The sofa base (underside 0.13 m) is in its overhang slice |
| `house_slices_lp1475.npz` | The low-profile R1 before task L: LiDAR plane 0.1475 m, an overhang slice up to the robot top 0.15515 m. Use it with `HOUSE_SLICES=house_slices_lp1475.npz` |
| `house_slices.npz` | The old robot: LiDAR plane 0.19 m. Use it with `HOUSE_SLICES=house_slices.npz` |
| `slice_house.py` | Rebuilds slices from `Content/RobotData/house-meshes.json`: `slice_house.py OUT.npz .1275 .13515` |
| `make_variant.py` | Makes a copy of the code and data with some robot-parameters changed (e.g. the LiDAR mount). The sim's own files are not touched |
| `dbg_stuck.py`, `dbg_stuck2.py` | Old debugging helpers. Print the trace after N seconds |

## Running it

```sh
cd tests/house2d
../../.venv/bin/python run_house.py s0 1600 0 cam
```

- The sensor errors follow `Content/RobotData/sensor-errors.json`: **on** by default. `R1_SENSOR_ERRORS=off` turns them off.
- The seed sets the LiDAR noise and the IMU noise sequence. The board is the one in the JSON.
- `cam` turns on the emulated low-obstacle camera.
- `ovcam` (task M) also emulates the camera's band pass on the overhangs (`house_world.overhang_view`): an overhang's edge is
  marked 0.10-0.45 m ahead of the lens, within 40° of face-on, while driving forward, and the floor behind it is mapped as
  under it (`LidarGrid.add_overhang_shadow`), as `LidarBrain.observe` does with the real detector. Without it the sofa base in
  the default slices is caught only by stall marks.
- The brain stops itself at `TIME_BUDGET` (1500 s).

**Outputs** go to `$HOUSE2D_OUT` (default `<sim>/reports/house2d`):
- `LABEL.log`: targets, events and body blocks, with a `SETUP` line giving the controller tree, LiDAR mount and slices used;
- `LABEL.json`: the result;
- `map-LABEL/`: the robot's grid.

`LABEL.json` holds:
- `complete` and `message`;
- `rooms`: per room, entered and mapped-free fraction, scored with `tests/score_full_mapping.py`'s reference;
- `contacts`, `skips`, `body_blocks` and `bumps`;
- `max_err`: the localisation error;
- `stage_s`: seconds on frontier targets, on coverage targets, and on none;
- `first_coverage_s`: the end of the first LiDAR frontier stage;
- `unvisited_timeline`;
- `blockers`: the `mapping_complete()` conditions at the end, with `disconnected_safe_m2` (safe floor the planner cannot reach)
  and `unreached_enterable_m2` (the part of it the body could enter; only this counts against the 1.5 m² limit, task H);
- `covered_on_the_way_m`: for each "Target area covered on the way", the robot's distance to that target (beyond 1.2 m the
  camera did not pass it: the target only dropped out of reach, which task H stopped counting as covered);
- `unvisited_by_room` and `unvisited_clusters`;
- `contact_phases`, `contact_phase_s` and `longest_contact_phase_s`: how often and how long the robot was in its bumper-contact
  phase (task G: a robot that cannot get off a contact used to stay in it until the time budget);
- `reverse_audit`: `sim2d.reverse_audit` on the true path (no reverse off the driven trail, episodes within the reverse limit).

All of these are evaluation only. The brain is not changed.

## Robot-parameter variants (e.g. the LiDAR mount)

```sh
../../.venv/bin/python make_variant.py ../../reports/house2d/variants/x0.06 \
  '{"lidar_mount_m":[0.06,0,0.1275],"lidar_turret_xy_m":[0.06,0]}'
V=../../reports/house2d/variants/x0.06
CTRL=$V/controller TESTS=$V/tests ../../.venv/bin/python run_house.py x006_s0 1600 0 cam
```

`variant.json` in the variant folder lists the overrides and the sha256 of every copied file. The code is byte-identical to
the sim's. Remake the variant after any code change.

## Limits

- The LiDAR sees only the LiDAR-plane slice (tall walls and furniture).
- Low objects reach the robot only through the bumper and the emulated camera.
- Overhangs (the blind band) touch only the LiDAR turret.
- There are no rugs, thresholds or caster catches. It is not a physics simulation: use the Unreal benchmark
  (`tests/full_mapping_run.py`) for that.
