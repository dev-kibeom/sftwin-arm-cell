"""Manual dynamic-control articulation discovery probe for Isaac Play mode."""

CANDIDATE_PATHS = (
    "/m0609_robotiq_2f85",
    "/m0609_robotiq_2f85/root_joint",
    "/m0609_robotiq_2f85/base_link",
    "/m0609_robotiq_2f85/base",
    "/m0609_robotiq_2f85/link_1",
)


def run_probe():
    """Print the legacy runtime articulation report without changing the stage."""
    import omni.usd
    from omni.isaac.dynamic_control import _dynamic_control

    stage = omni.usd.get_context().get_stage()
    dc = _dynamic_control.acquire_dynamic_control_interface()

    print("=== Dynamic-Control Articulation Probe ===")
    print("is_simulating:", dc.is_simulating())

    if not dc.is_simulating():
        raise RuntimeError(
            "Press Play, wait at least one physics frame, then run this probe."
        )

    hits = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        try:
            obj_type = dc.peek_object_type(path)
        except Exception:
            continue
        if obj_type != _dynamic_control.ObjectType.OBJECT_NONE:
            print(f"{path} -> {obj_type}")
        if obj_type == _dynamic_control.ObjectType.OBJECT_ARTICULATION:
            hits.append(path)

    print("\\n=== Articulations ===")
    if not hits:
        print("NO runtime articulation object found.")
    else:
        for path in hits:
            print(path)
            handle = dc.get_articulation(path)
            print("  handle valid:", handle != _dynamic_control.INVALID_HANDLE)
            if handle != _dynamic_control.INVALID_HANDLE:
                print("  runtime path:", dc.get_articulation_path(handle))
                print("  dof count:", dc.get_articulation_dof_count(handle))

    print("\\n=== Candidate paths explicitly checked ===")
    for path in CANDIDATE_PATHS:
        try:
            print(path, "->", dc.peek_object_type(path))
        except Exception as exc:
            print(path, "-> probe failed:", exc)
