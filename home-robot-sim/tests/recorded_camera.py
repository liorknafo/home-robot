"""Recorded camera fixtures and the camera mount they were recorded with.

The image fixtures under tests/fixtures were recorded in Unreal. slipper-approach and slipper-side-approach are still the
2026-10-03 recordings, made before the low-profile redesign with the camera at RECORDED_CAMERA_MOUNT (x, y, height above
the floor, metres; base_link at BASE_Z). floor-rejection and floor_parallax are the 2026-10-09 (bay B10 phase 2) recordings
at the low-profile mount (0.117, 0, 0.1075); bedroom-edges was re-recorded on 2026-10-11 (task F) at the bumper-band mount
(0.146, 0, 0.08). A fixture's inputs.json names its mount in 'camera_mount_m' (fixture_mount), and a fixture without that key
is a RECORDED_CAMERA_MOUNT recording. The detectors take their extrinsics from geometry.CAMERA_MOUNT (robot-parameters.json
camera_mount_m), which is now the bumper-band mount (0.146, 0, 0.08). Replaying
images with another mount than the one they were taken with would put every pixel ray at the wrong height, so every
replay uses the mount its images were taken with: recorded_mount() swaps it in for mono.camera() (used by MonoObserver and
FloorParallax) and FloorParallax's plausible camera-height band, and restores the current mount afterwards. The
current mount itself is covered by the synthetic tests (test_floor_parallax, test_mono_obstacles), which render at
geometry.CAMERA_MOUNT. Re-recording the fixtures at the new mount needs the Unreal simulator.
"""
from contextlib import contextmanager
import autonomy.mono as mono
import autonomy.floor_parallax as floor_parallax

RECORDED_CAMERA_MOUNT = (.117, 0., .15)


@contextmanager
def recorded_mount(mount=RECORDED_CAMERA_MOUNT):
    """Within the block, mono.camera() and FloorParallax use `mount` (the recording's camera extrinsics)."""
    saved = mono.CAMERA_MOUNT, floor_parallax.CAMERA_HEIGHT_RANGE
    mono.CAMERA_MOUNT = tuple(mount)
    floor_parallax.CAMERA_HEIGHT_RANGE = (mount[2] - .03, mount[2] + .03)
    try:
        yield
    finally:
        mono.CAMERA_MOUNT, floor_parallax.CAMERA_HEIGHT_RANGE = saved


def fixture_mount(inputs):
    """The camera mount a fixture's images were recorded with: its inputs.json 'camera_mount_m', else RECORDED_CAMERA_MOUNT."""
    return tuple(inputs.get('camera_mount_m', RECORDED_CAMERA_MOUNT))
