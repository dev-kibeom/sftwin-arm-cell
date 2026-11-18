"""Physical indexed pallet conveyor and split-belt outfeed owned by Tape Cell.

A force-limited linear pallet drive represents the conveyor's drive chain.
Rolls are free dynamic bodies transported by contact with pallet pockets/belts.
Only initial spawning and stopped reset author material poses.
"""
import math


class Conveyor:
    def __init__(self, stage, configuration, roll):
        from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf
        self.stage,self.configuration,self.roll=stage,configuration,roll
        self.layout=configuration['cell']['tape_roll']
        self.settings=configuration['cell']['conveyor']
        self.root='/World/TapeRollCell/Transport'
        self.state='idle';self.detail='Supply를 눌러 가득 찬 카트리지를 공급하세요.'
        self.queued=self.accepted=self.finished=0
        self.number=0;self.ordinal=0;self.active_roll_paths=[];self.history=[]
        self.carrier_path=None;self.drive=None;self.settle=0.;self.elapsed=0.
        self.output_path=None;self._speed=0.;self._target=0.;self._command=0.
        self.Gf=Gf
        self.material=UsdShade.Material.Define(stage,'/World/TapeRollCell/ConveyorContact')
        api=UsdPhysics.MaterialAPI.Apply(self.material.GetPrim())
        api.CreateStaticFrictionAttr(roll.static_friction);api.CreateDynamicFrictionAttr(roll.dynamic_friction)
        api.CreateRestitutionAttr(0.)
        self.surfaces=[]
        for item in self.layout['static_boxes']:
            if item['id'].endswith(('/InfeedBelt','/OutfeedLeft','/OutfeedRight')):
                prim=stage.GetPrimAtPath(item['id'])
                rb=UsdPhysics.RigidBodyAPI.Apply(prim);rb.CreateKinematicEnabledAttr(True)
                surface=PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
                surface.CreateSurfaceVelocityLocalSpaceAttr(False)
                surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0.))
                self.surfaces.append((item['id'],surface))

    def _box(self,path,position,size,angle=0.):
        from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema
        cube=UsdGeom.Cube.Define(self.stage,path);cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*position))
        if angle:cube.AddRotateXOp().Set(angle)
        cube.AddScaleOp().Set(self.Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([self.Gf.Vec3f(.6,.28,.08)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(.0002);contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(self.material,materialPurpose='physics')
        return cube

    def enqueue_cartridges(self,count):
        if isinstance(count,bool) or not isinstance(count,int) or not 0<count<=self.settings['max_cartridges']:
            return dict(accepted=False,detail=f"카트리지 수는 1–{self.settings['max_cartridges']} 범위입니다.")
        if self.state!='idle':return dict(accepted=False,detail='현재 공급이 끝나거나 Stop한 뒤 다시 요청하세요.')
        self.queued=self.accepted=count;self.finished=0;self.state='waiting_spawn'
        self.detail=f'{count}개 카트리지 공급 요청 수락 (각 {self.layout["cartridge"]["slot_count"]}롤)'
        return dict(accepted=True,detail=self.detail)

    def _spawn(self):
        from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade
        from tape_roll_cell.fixtures import cartridge_bed_bottom_z,CARTRIDGE_BED_THICKNESS_M
        self.number+=1;self.ordinal=0;self.queued-=1
        cfg=self.settings;pitch=self.layout['cartridge']['slot_spacing_m'];n=self.layout['cartridge']['slot_count']
        x=cfg['inlet_front_x_m'];y=self.layout['supply_center_m'][1];z=self.layout['supply_center_m'][2]
        self.carrier_path=f'{self.root}/Carrier{self.number:04d}'
        carrier=UsdGeom.Xform.Define(self.stage,self.carrier_path)
        carrier.AddTranslateOp().Set(self.Gf.Vec3d(x,y,0.))
        UsdPhysics.RigidBodyAPI.Apply(carrier.GetPrim());UsdPhysics.MassAPI.Apply(carrier.GetPrim()).CreateMassAttr(3.)
        self._box(self.carrier_path+'/Bed',((n-1)*pitch/2,0.,cartridge_bed_bottom_z(self.layout,self.roll)+CARTRIDGE_BED_THICKNESS_M/2),
            ((n-1)*pitch+self.roll.width_m+.025,2*self.roll.outer_radius_m+.025,CARTRIDGE_BED_THICKNESS_M))
        # Tangent V faces mechanically center every roll on the known Y/Z datum.
        # The bore remains open for horizontal Tool insertion.
        offset=(self.roll.outer_radius_m+.006)/math.sqrt(2.)
        for side in (-1,1):
            self._box(self.carrier_path+f'/Cradle{int(side>0)}',
                ((n-1)*pitch/2,side*offset,z-offset),
                ((n-1)*pitch+self.roll.width_m+.025,.06,.012),side*45.)
        for sign in (-1,1):
            self._box(self.carrier_path+f'/Guide{int(sign>0)}',((n-1)*pitch/2,sign*(self.roll.outer_radius_m+.009),z-self.roll.outer_radius_m+.014),
                ((n-1)*pitch+self.roll.width_m+.025,.01,.04))
        for i in range(n):
            for side in (-1,1):
                for sign in (-1,1):
                    self._box(self.carrier_path+f'/Pocket{i}_{int(side>0)}_{int(sign>0)}',
                        (i*pitch+side*(self.roll.width_m/2+.0025),sign*.05,z-.05),(.004,.008,.008))
        joint=UsdPhysics.PrismaticJoint.Define(self.stage,self.carrier_path+'/Rail')
        joint.CreateBody1Rel().SetTargets([self.carrier_path]);joint.CreateAxisAttr('X')
        joint.CreateLocalPos0Attr(self.Gf.Vec3f(0.,y,0.));joint.CreateLocalPos1Attr(self.Gf.Vec3f(0.))
        joint.CreateLowerLimitAttr(cfg['minimum_front_x_m']);joint.CreateUpperLimitAttr(cfg['inlet_front_x_m']+.1)
        self.drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
        self.drive.CreateTypeAttr('force');self.drive.CreateStiffnessAttr(cfg['drive_stiffness_n_m'])
        self.drive.CreateDampingAttr(cfg['drive_damping_ns_m']);self.drive.CreateMaxForceAttr(cfg['drive_force_n'])
        self.drive.CreateTargetPositionAttr(x);self._command=x;self._speed=0.
        self.active_roll_paths=[]
        faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        for i in range(n):
            path=f'{self.root}/Loads{self.number:04d}/Roll{i:03d}'
            body=UsdGeom.Xform.Define(self.stage,path);body.AddTranslateOp().Set(self.Gf.Vec3d(x+i*pitch,y,z))
            body.AddRotateYOp().Set(90.);UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            mass=UsdPhysics.MassAPI.Apply(body.GetPrim());mass.CreateMassAttr(self.roll.mass_kg)
            mass.CreateCenterOfMassAttr(self.Gf.Vec3f(0.));mass.CreateDiagonalInertiaAttr(self.Gf.Vec3f(*self.roll.inertia()))
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
            rb.CreateSolverPositionIterationCountAttr(32);rb.CreateSolverVelocityIterationCountAttr(8)
            # Robot bodies already report every Robot/material contact. Pallet
            # support contacts are not consumed by the supply controller; do
            # not flood both Tool and Cell observers with all stored-roll pairs.
            for j,vertices in enumerate(self.roll.convex_segments()):
                mesh=UsdGeom.Mesh.Define(self.stage,path+f'/Sector{j:03d}')
                mesh.CreatePointsAttr([self.Gf.Vec3f(*p) for p in vertices]);mesh.CreateFaceVertexCountsAttr([4]*6)
                mesh.CreateFaceVertexIndicesAttr([v for face in faces for v in face]);mesh.CreateSubdivisionSchemeAttr('none')
                UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
                contact=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
                contact.CreateContactOffsetAttr(.0002);contact.CreateRestOffsetAttr(0.)
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(self.material,materialPurpose='physics')
                mesh.CreateDisplayColorAttr([self.Gf.Vec3f(.65,.43,.2)])
                paper=UsdShade.Material.Get(self.stage,'/World/TapeRollCell/Presentation/Looks/paper')
                if paper:UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(paper)
            self.active_roll_paths.append(path)
        self.history.append((self.number,list(self.active_roll_paths)))
        self._target=self.layout['supply_center_m'][0];self.state='infeed';self.elapsed=self.settle=0.
        self.detail='가득 찬 카트리지 이송 중'

    def update(self):
        # Author/remove USD objects from the application update, not a solver callback.
        if self.state=='waiting_spawn':self._spawn()
        elif self.state=='unloading':
            self.stage.RemovePrim(self.carrier_path);self.carrier_path=None
            self.active_roll_paths=[];self.finished+=1
            self.state='waiting_spawn' if self.queued else 'idle'
            self.detail='다음 카트리지 대기' if self.queued else '공급 완료'

    def _measure(self,path):
        from omni.isaac.dynamic_control import _dynamic_control as api
        dc=api.acquire_dynamic_control_interface();handle=dc.get_rigid_body(path)
        if handle==api.INVALID_HANDLE:return None
        pose=dc.get_rigid_body_pose(handle);vel=dc.get_rigid_body_linear_velocity(handle)
        angular=dc.get_rigid_body_angular_velocity(handle)
        return pose,vel,angular

    @property
    def dock_ready(self):return self.state=='docked'

    @property
    def current_ordinal(self):return self.ordinal

    def release_to_outfeed(self,path):
        if not self.dock_ready:raise RuntimeError('Conveyor is not docked')
        # A settled release may sleep during Retreat. Surface velocity changes
        # alone do not wake the payload; resume contact solving without writing
        # its pose or velocity. Belt friction must produce the actual motion.
        self._wake_roll(path)
        self.output_path=path;self.state='outfeed';self.elapsed=0.

    def _wake_roll(self,path):
        from omni.isaac.dynamic_control import _dynamic_control as api
        dc=api.acquire_dynamic_control_interface();handle=dc.get_rigid_body(path)
        if handle==api.INVALID_HANDLE:raise RuntimeError('Released payload is unavailable')
        dc.wake_up_rigid_body(handle)

    def _surface(self,infeed=0.,outfeed=0.):
        for path,api in self.surfaces:
            api.GetSurfaceVelocityAttr().Set(self.Gf.Vec3f(infeed,0.,0.) if path.endswith('/InfeedBelt') else self.Gf.Vec3f(0.,-outfeed,0.))

    def on_physics_step(self,dt):
        if not math.isfinite(dt) or dt<=0:return
        if self.state not in ('infeed','indexing','returning','outfeed'):
            self._surface();return
        self.elapsed+=dt
        if self.elapsed>self.settings['movement_timeout_s']:
            self.state='failed';self.detail='컨베이어 도착/배출 위치를 물리 관측으로 확인하지 못했습니다.'
            self._surface();return
        if self.state=='outfeed':
            self._surface(outfeed=self.settings['outfeed_speed_m_s'])
            row=self._measure(self.output_path)
            if row and row[0].p.y <= self.layout['place']['first_center_xy_m'][1]-self.settings['outfeed_pitch_m']:
                self._surface();self.output_path=None
                if self.ordinal+1 < len(self.active_roll_paths):
                    self.ordinal+=1;self._target=self.layout['supply_center_m'][0]-self.ordinal*self.layout['cartridge']['slot_spacing_m']
                    self.state='indexing'
                else:self._target=self.settings['inlet_front_x_m'];self.state='returning'
                self.elapsed=self.settle=0.
            return
        row=self._measure(self.carrier_path)
        if row is None:return
        delta=self._target-self._command
        desired=math.copysign(min(self.settings['speed_m_s'],math.sqrt(2*self.settings['acceleration_m_s2']*abs(delta))),delta) if delta else 0.
        change=self.settings['acceleration_m_s2']*dt
        self._speed+=max(-change,min(change,desired-self._speed))
        step=self._speed*dt
        self._command=self._target if abs(step)>=abs(delta) else self._command+step
        self.drive.GetTargetPositionAttr().Set(self._command);self._surface(infeed=self._speed)
        pose,velocity,angular=row
        settled=(self._command==self._target and abs(self._speed)<.008 and
                 abs(pose.p.x-self._target)<self.settings['position_tolerance_m'] and abs(velocity.x)<.008)
        if settled and self.state!='returning':
            # A full cartridge must retain every unprocessed roll, not just
            # present one surviving roll while the remaining inventory is lost.
            for index in range(self.ordinal,len(self.active_roll_paths)):
                roll=self._measure(self.active_roll_paths[index])
                if roll is None:
                    settled=False;break
                datum=list(self.layout['supply_center_m'])
                datum[0]+=(index-self.ordinal)*self.layout['cartridge']['slot_spacing_m']
                p,v,w=roll
                axis_x=2*(p.r.x*p.r.z+p.r.w*p.r.y)
                settled=settled and abs(axis_x)>math.cos(.03)
                settled=settled and sum((a-b)**2 for a,b in zip((p.p.x,p.p.y,p.p.z),datum))<self.settings['position_tolerance_m']**2
                settled=settled and v.x*v.x+v.y*v.y+v.z*v.z<.0001 and w.x*w.x+w.y*w.y+w.z*w.z<.04
                if not settled:break
        self.settle=self.settle+dt if settled else 0.
        if self.settle>=self.settings['settle_s']:
            self._surface();self.elapsed=self.settle=0.
            if self.state=='returning':
                self.state='unloading'
            else:self.state='docked';self.detail=f'고정 Pick 위치 도착 · 슬롯 {self.ordinal+1}'

    def resetstopped(self):
        self._surface()
        if self.stage.GetPrimAtPath(self.root):self.stage.RemovePrim(self.root)
        self.state='idle';self.detail='Supply 대기';self.queued=self.accepted=self.finished=0
        self.active_roll_paths=[];self.history=[];self.carrier_path=None;self.drive=None;self.output_path=None

    def close(self):self._surface()
