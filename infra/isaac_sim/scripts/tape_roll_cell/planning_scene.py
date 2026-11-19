"""Measured annular materials in MoveIt's scene; planning attachments are not PhysX joints."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation


class ConfiguredPlanningScene:
    def __init__(self,node):
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import GetPlanningScene
        self.node=node
        self.publisher=node.create_publisher(PlanningScene,'/planning_scene',10)
        self.client=node.create_client(GetPlanningScene,'/get_planning_scene')
        self.acm=None;self.query=None;self.attached=None;self.relation=None
        self._world_ids=set()
        self._frame_transform=None

    def _base_acm(self, frame):
        if self.acm is None and self.query is None and self.client.service_is_ready():
            from moveit_msgs.srv import GetPlanningScene
            from moveit_msgs.msg import PlanningSceneComponents
            request=GetPlanningScene.Request();request.components.components=(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.TRANSFORMS | PlanningSceneComponents.ROBOT_STATE)
            self.query=self.client.call_async(request)
            def received(future):
                try:
                    scene=future.result().scene
                    # MoveIt must acknowledge the producer frame before it can
                    # interpret these collision poses. TF arrival is asynchronous.
                    model_frames=(scene.robot_state.joint_state.header.frame_id,
                                  scene.robot_state.multi_dof_joint_state.header.frame_id)
                    transform=next((t for t in scene.fixed_frame_transforms if t.child_frame_id==frame),None)
                    if transform is None:
                        reverse=next((t for t in scene.fixed_frame_transforms
                            if t.header.frame_id==frame and t.child_frame_id in model_frames),None)
                        if reverse is not None:
                            # GetPlanningScene may return the model's pose in
                            # world; collision poses need the opposite mapping.
                            transform=copy.deepcopy(reverse)
                            transform.header.frame_id=reverse.child_frame_id
                            transform.child_frame_id=frame
                            q=reverse.transform.rotation;t=reverse.transform.translation
                            inverse=Rotation.from_quat([q.x,q.y,q.z,q.w]).inv()
                            translation=inverse.apply([-t.x,-t.y,-t.z])
                            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=translation.tolist()
                            (transform.transform.rotation.x,transform.transform.rotation.y,
                             transform.transform.rotation.z,transform.transform.rotation.w)=inverse.as_quat().tolist()
                    if frame in model_frames or transform is not None:
                        self.acm=scene.allowed_collision_matrix
                        self._frame_transform=transform
                except Exception:
                    # Retry an unavailable scene without sending uninterpretable poses.
                    pass
                finally:self.query=None
            self.query.add_done_callback(received)
        return self.acm

    @staticmethod
    def _annulus(identity,material,position,orientation,frame):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        obj=CollisionObject();obj.id=identity;obj.header.frame_id=frame;obj.operation=obj.ADD
        obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=position
        obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w=orientation
        inner,outer,width=material['inner_radius_m'],material['outer_radius_m'],material['width_m']
        segments=int(material.get('segments',64))
        for index in range(segments):
            angle=2*math.pi*index/segments
            box=SolidPrimitive();box.type=box.BOX
            box.dimensions=[outer-inner,2*outer*math.tan(math.pi/segments),width]
            pose=Pose();radius=(outer+inner)/2
            pose.position.x,pose.position.y=radius*math.cos(angle),radius*math.sin(angle)
            pose.orientation.z,pose.orientation.w=math.sin(angle/2),math.cos(angle/2)
            obj.primitives.append(box);obj.primitive_poses.append(pose)
        return obj

    def publish(self,packet):
        from moveit_msgs.msg import PlanningScene,AttachedCollisionObject,CollisionObject,AllowedCollisionEntry
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        base=self._base_acm(packet['frame_id'])
        if base is None:
            return
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        relation=packet.get('accepted_relation');selected=packet.get('selected_slot_id')
        rolls=packet['rolls'];finger_links=packet['finger_links']
        previous_attached=self.attached
        current_ids={row['id'] for row in rolls}|{box['id'] for box in packet.get('static_boxes',[])}
        selected_row=next((row for row in rolls if row['slot_id']==selected),None)
        if packet.get('holding_state')=='held' and selected_row is not None and relation is not None:
            self.attached=selected_row['id'];self.relation=np.array(relation).reshape(4,4)
        if self.attached and (packet.get('holding_state')=='released' or not selected):
            detach=AttachedCollisionObject();detach.link_name='sf_grasp_tcp';detach.object.id=self.attached;detach.object.operation=CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(detach)
            self.attached=None;self.relation=None
        current_ids.discard(self.attached)
        for identity in self._world_ids-current_ids:
            remove=CollisionObject();remove.id=identity;remove.header.frame_id=packet['frame_id'];remove.operation=remove.REMOVE
            scene.world.collision_objects.append(remove)
        self._world_ids=current_ids
        for row in rolls:
            if row['id']==self.attached:
                if self.attached==previous_attached:
                    continue
                attached=AttachedCollisionObject();attached.link_name='sf_grasp_tcp';attached.touch_links=finger_links
                attached.object=self._annulus(row['id'],packet['material'],self.relation[:3,3].tolist(),Rotation.from_matrix(self.relation[:3,:3]).as_quat().tolist(),'sf_grasp_tcp')
                scene.robot_state.attached_collision_objects.append(attached)
            else:
                scene.world.collision_objects.append(self._annulus(row['id'],packet['material'],row['position'],row['orientation_xyzw'],packet['frame_id']))
        for box in packet.get('static_boxes',[]):
            obj=CollisionObject();obj.id=box['id'];obj.header.frame_id=packet['frame_id'];obj.operation=obj.ADD
            obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=box['position']
            q=box.get('orientation_xyzw',[0.,0.,0.,1.])
            obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w=q
            primitive=SolidPrimitive();primitive.type=primitive.BOX;primitive.dimensions=box['dimensions']
            local=Pose();local.orientation.w=1.;obj.primitives=[primitive];obj.primitive_poses=[local]
            scene.world.collision_objects.append(obj)
        if base is not None:
            acm=copy.deepcopy(base)
            names=list(acm.entry_names)
            dynamic=[row['id'] for row in rolls]+finger_links+packet.get('mount_links',[])+[box['id'] for box in packet.get('static_boxes',[])]
            for name in dynamic:
                if name not in names:names.append(name)
            matrix=[[False]*len(names) for _ in names]
            for i,old in enumerate(acm.entry_names):
                for j,other in enumerate(acm.entry_names):matrix[names.index(old)][names.index(other)]=acm.entry_values[i].enabled[j]
            def allow(a,b,value):
                i,j=names.index(a),names.index(b);matrix[i][j]=matrix[j][i]=value
            if packet.get('mount_object') in names:
                for link in packet.get('mount_links',[]):allow(link,packet['mount_object'],True)
            for row in rolls:
                for finger in finger_links:allow(row['id'],finger,row['slot_id']==selected)
                for box in packet.get('static_boxes',[]):
                    prefix=row.get('support_prefix')
                    own_support=bool(prefix and box['id'].startswith(prefix+'/'))
                    allow(row['id'],box['id'],row['slot_id']==selected and own_support)
            acm.entry_names=names;acm.entry_values=[]
            for values in matrix:
                entry=AllowedCollisionEntry();entry.enabled=values;acm.entry_values.append(entry)
            scene.allowed_collision_matrix=acm
        if self._frame_transform is not None:
            from tf2_geometry_msgs import do_transform_pose
            # Each Motion/MoveGroup scene receives TF independently. Use the
            # model frame so none needs an external frame to apply this diff.
            for obj in scene.world.collision_objects:
                if obj.header.frame_id==packet['frame_id']:
                    if obj.operation!=obj.REMOVE:
                        obj.pose=do_transform_pose(obj.pose,self._frame_transform)
                    obj.header.frame_id=self._frame_transform.header.frame_id
        self.publisher.publish(scene)
