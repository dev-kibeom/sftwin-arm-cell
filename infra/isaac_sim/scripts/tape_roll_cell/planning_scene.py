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

    def _base_acm(self):
        if self.acm is None and self.query is None and self.client.service_is_ready():
            from moveit_msgs.srv import GetPlanningScene
            from moveit_msgs.msg import PlanningSceneComponents
            request=GetPlanningScene.Request();request.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.query=self.client.call_async(request)
            def received(future):
                try:self.acm=future.result().scene.allowed_collision_matrix
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
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        relation=packet.get('accepted_relation');selected=packet.get('selected_slot_id')
        rolls=packet['rolls'];finger_links=packet['finger_links']
        selected_row=next((row for row in rolls if row['slot_id']==selected),None)
        if packet.get('holding_state')=='held' and selected_row is not None and relation is not None:
            self.attached=selected_row['id'];self.relation=np.array(relation).reshape(4,4)
        if self.attached and (packet.get('holding_state')=='released' or not selected):
            detach=AttachedCollisionObject();detach.link_name='sf_grasp_tcp';detach.object.id=self.attached;detach.object.operation=CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(detach)
            self.attached=None;self.relation=None
        for row in rolls:
            if row['id']==self.attached:
                remove=CollisionObject();remove.id=row['id'];remove.header.frame_id='world';remove.operation=remove.REMOVE
                scene.world.collision_objects.append(remove)
                attached=AttachedCollisionObject();attached.link_name='sf_grasp_tcp';attached.touch_links=finger_links
                attached.object=self._annulus(row['id'],packet['material'],self.relation[:3,3].tolist(),Rotation.from_matrix(self.relation[:3,:3]).as_quat().tolist(),'sf_grasp_tcp')
                scene.robot_state.attached_collision_objects.append(attached)
            else:
                scene.world.collision_objects.append(self._annulus(row['id'],packet['material'],row['position'],row['orientation_xyzw'],packet['frame_id']))
        for box in packet.get('static_boxes',[]):
            obj=CollisionObject();obj.id=box['id'];obj.header.frame_id=packet['frame_id'];obj.operation=obj.ADD
            obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=box['position'];obj.pose.orientation.w=1.
            primitive=SolidPrimitive();primitive.type=primitive.BOX;primitive.dimensions=box['dimensions']
            local=Pose();local.orientation.w=1.;obj.primitives=[primitive];obj.primitive_poses=[local]
            scene.world.collision_objects.append(obj)
        base=self._base_acm()
        if base is not None:
            acm=copy.deepcopy(base)
            names=list(acm.entry_names)
            dynamic=[row['id'] for row in rolls]+finger_links+[box['id'] for box in packet.get('static_boxes',[])]
            for name in dynamic:
                if name not in names:names.append(name)
            matrix=[[False]*len(names) for _ in names]
            for i,old in enumerate(acm.entry_names):
                for j,other in enumerate(acm.entry_names):matrix[names.index(old)][names.index(other)]=acm.entry_values[i].enabled[j]
            def allow(a,b,value):
                i,j=names.index(a),names.index(b);matrix[i][j]=matrix[j][i]=value
            for row in rolls:
                for finger in finger_links:allow(row['id'],finger,row['slot_id']==selected)
                for box in packet.get('static_boxes',[]):
                    own_support=f"Cartridge{int(row['slot_id'].split('_')[-1]):03d}Floor" in box['id']
                    allow(row['id'],box['id'],row['slot_id']==selected and own_support)
            acm.entry_names=names;acm.entry_values=[]
            for values in matrix:
                entry=AllowedCollisionEntry();entry.enabled=values;acm.entry_values.append(entry)
            scene.allowed_collision_matrix=acm
        self.publisher.publish(scene)
