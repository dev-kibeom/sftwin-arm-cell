"""Pure first-pass reachability measurements and build-time reporting."""

import numpy as np


def planar_distance_xy(a, b):
    dx = float(b[0]) - float(a[0])
    dy = float(b[1]) - float(a[1])
    return float(np.sqrt(dx * dx + dy * dy))


def spatial_distance_xyz(a, b):
    dx = float(b[0]) - float(a[0])
    dy = float(b[1]) - float(a[1])
    dz = float(b[2]) - float(a[2])
    return float(np.sqrt(dx * dx + dy * dy + dz * dz))


def report_reach(name, planar_m, spatial_m, report=print):
    comfortable_planar_reach = 0.75
    max_nominal_planar_reach = 0.90
    if planar_m > max_nominal_planar_reach:
        tag, note = "FAIL", "outside nominal planar reach"
    elif planar_m > comfortable_planar_reach:
        tag, note = "WARN", "near reach boundary; verify with IK"
    else:
        tag, note = "OK", "reasonable first-pass layout"
    report(
        f">>> [KINEMATIC {tag}] {name}: planar={planar_m:.3f} m, spatial={spatial_m:.3f} m - {note}"
    )


def run_reachability_check(context, report=print):
    robot_base_xyz = np.array([context.robot_x, context.robot_y, 0.68], dtype=float)
    raw_pick_xyz = np.array(
        [context.raw_slot_x, context.slot_y, context.amr_tray_z + 0.12], dtype=float
    )
    cnc_place_xyz = np.array(
        [context.cnc_x - 0.08, context.cnc_y - 0.49, 0.88], dtype=float
    )
    finished_place_xyz = np.array(
        [context.done_slot_x, context.slot_y, context.amr_tray_z + 0.10], dtype=float
    )
    report("\n>>> [KINEMATIC CHECK] First-pass task-point distances")
    for label, target in [
        ("Robot -> AMR RAW pick", raw_pick_xyz),
        ("Robot -> CNC place", cnc_place_xyz),
        ("Robot -> AMR FINISHED place", finished_place_xyz),
    ]:
        report_reach(
            label,
            planar_distance_xy(robot_base_xyz, target),
            spatial_distance_xyz(robot_base_xyz, target),
            report,
        )
    report(
        ">>> [NOTE] Final reachability still requires TCP-aware IK, joint-limit and collision checks.\n"
    )
