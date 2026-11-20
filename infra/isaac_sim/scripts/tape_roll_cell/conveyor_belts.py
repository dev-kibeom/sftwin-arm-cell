"""Isaac Sim native Conveyor nodes driven by the Cell's physics-step commands."""


class ConveyorBelts:
    def __init__(self, stage, static_boxes, *, graph_path='/World/TapeRollCell/ConveyorGraph'):
        import omni.graph.core as og
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import PhysxSchema, UsdPhysics

        if not enable_extension('isaacsim.asset.gen.conveyor'):
            raise RuntimeError('Tape Cell requires the isaacsim.asset.gen.conveyor extension')
        self._og = og
        self._path = graph_path
        if stage.GetPrimAtPath(self._path):
            stage.RemovePrim(self._path)
        nodes = [('Command', 'omni.graph.action.OnImpulseEvent')]
        values = [('Command.inputs:onlyPlayback', False)]
        connections = []
        self._speeds = []
        self._surface_enables = []
        for item in static_boxes:
            path = item['id']
            name = path.rsplit('/', 1)[-1]
            if name not in ('InfeedBelt', 'OutfeedLeft', 'OutfeedRight'):
                continue
            prim = stage.GetPrimAtPath(path)
            body = UsdPhysics.RigidBodyAPI.Apply(prim)
            body.CreateKinematicEnabledAttr(True)
            surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            surface.CreateSurfaceVelocityLocalSpaceAttr(False)
            self._surface_enables.append((surface.GetSurfaceVelocityEnabledAttr(), name == 'InfeedBelt'))
            infeed = name == 'InfeedBelt'
            nodes.append((name, 'isaacsim.asset.gen.conveyor.IsaacConveyor'))
            values.extend([
                (name+'.inputs:conveyorPrim', [path]),
                (name+'.inputs:direction', [1., 0., 0.] if infeed else [0., -1., 0.]),
                (name+'.inputs:velocity', 0.),
                (name+'.inputs:enabled', True),
                (name+'.inputs:animateTexture', False),
                (name+'.inputs:curved', False),
            ])
            connections.append(('Command.outputs:execOut', name+'.inputs:onStep'))
            self._speeds.append((name, infeed, prim))
        self._graph, _, _, _ = og.Controller.edit(
            dict(graph_path=self._path, evaluator_name='execution',
                 pipeline_stage=og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND),
            {og.Controller.Keys.CREATE_NODES: nodes,
             og.Controller.Keys.SET_VALUES: values,
             og.Controller.Keys.CONNECT: connections})
        self._speeds = [(og.Controller.attribute(self._path+'/'+name+'.inputs:velocity'),
                         og.Controller.attribute(self._path+'/'+name+'.inputs:direction'), infeed, prim)
                        for name, infeed, prim in self._speeds]
        self._trigger = og.Controller.attribute(self._path+'/Command.state:enableImpulse')
        self.set_speeds()

    def set_speeds(self, *, infeed=0., outfeed=0.):
        from pxr import Gf, Usd, UsdGeom

        for speed, direction, is_infeed, prim in self._speeds:
            # Isaac 5.1's native actuation uses world directions. Resolve the
            # stopped equipment placement explicitly, ignoring box scale.
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            axis = Gf.Vec3d(1., 0., 0.) if is_infeed else Gf.Vec3d(0., -1., 0.)
            direction.set(Gf.Transform(matrix).GetRotation().TransformDir(axis))
            speed.set(infeed if is_infeed else outfeed)
        # Evaluate only this graph, including zero commands on close/Stop.
        # A playback/render tick would delay or miss the physics-step command,
        # especially when rendering is throttled or the timeline is stopped.
        self._trigger.set(True)
        self._og.Controller.evaluate_sync(self._graph)
        # Isaac 5.1 can retain its last surface velocity for small speed
        # changes, including a slow ramp to zero. Make zero an exact stop
        # without taking velocity-vector actuation away from the native node.
        for enabled, is_infeed in self._surface_enables:
            enabled.Set((infeed if is_infeed else outfeed) != 0.)
