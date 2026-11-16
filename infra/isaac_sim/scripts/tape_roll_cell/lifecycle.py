"""Timeline Stop resets Cell-owned material and inventory, never a playing pick."""


def bind_stop_reset(stage, roll_paths, supply):
    import omni.timeline
    from pxr import Gf, UsdGeom, UsdPhysics
    timeline = omni.timeline.get_timeline_interface()
    initial = [(path, UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetLocalTransformation())
               for path in roll_paths]

    def reset(event):
        if event.type != int(omni.timeline.TimelineEventType.STOP):
            return
        if not timeline.is_stopped():
            raise RuntimeError('Tape Cell reset requires stopped simulation')
        for path, matrix in initial:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                raise RuntimeError(f'Tape Cell material missing during reset: {path}')
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix)
            body = UsdPhysics.RigidBodyAPI(prim)
            body.CreateVelocityAttr(Gf.Vec3f(0.))
            body.CreateAngularVelocityAttr(Gf.Vec3f(0.))
        supply.reset()

    return timeline.get_timeline_event_stream().create_subscription_to_pop(reset, name='tape_cell_stop_reset')
