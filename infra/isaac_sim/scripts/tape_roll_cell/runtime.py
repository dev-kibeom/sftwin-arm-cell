"""Tape Cell device entrypoints selected by the common Isaac workflow."""


def snapshot_camera(*, configuration, namespace):
    # This Cell has a configured supply source and no camera/Vision producer.
    return None


def enable_operator_ui(*, configuration, namespace):
    import omni.ui as ui
    from omni.kit.viewport.utility import get_active_viewport
    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = '/World/TapeRollCell/Presentation/OverviewCamera'
    adapter=namespace['_material_supply_adapter']
    window=ui.Window('Tape Handling Cell',width=400,height=160)
    def start():
        adapter.sidecar.send(dict(op='start_cycle',target_id=configuration['orchestration']['mission_target_id'],
            quantity=sum(adapter.supply.snapshot()['occupied'])))
        adapter.sidecar.flush_outgoing()
    with window.frame:
        with ui.VStack():
            ui.Label('Tape Roll · Physical Real')
            ui.Label('주황: 카트리지 · 청록: 컨베이어 수령부')
            ui.Label('ROS와 Safety가 준비된 후 작업을 시작하세요.')
            ui.Button('남은 카트리지 처리',clicked_fn=start)
            ui.Label('진행 결과는 ROS Mission 로그에서 확인합니다.')
    namespace['cell_operator_window']=window


def graph_declaration(binding):
    nodes=[('Tick','omni.graph.action.OnPlaybackTick'),('Context','isaacsim.ros2.bridge.ROS2Context'),
        ('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),('Clock','isaacsim.ros2.bridge.ROS2PublishClock'),
        ('JointState','isaacsim.ros2.bridge.ROS2PublishJointState'),('JointCommand','isaacsim.ros2.bridge.ROS2SubscribeJointState'),
        ('Arm','isaacsim.core.nodes.IsaacArticulationController'),('PublishGripperStatus','isaacsim.ros2.bridge.ROS2Publisher')]
    values=[('Context.inputs:useDomainIDEnvVar',True),('Time.inputs:resetOnStop',True),('Clock.inputs:topicName','/clock'),
        ('JointState.inputs:topicName','/isaac/joint_states'),('JointState.inputs:targetPrim',[binding.articulation_root_path]),
        ('JointCommand.inputs:topicName','/joint_commands'),('Arm.inputs:targetPrim',[binding.articulation_root_path]),
        ('PublishGripperStatus.inputs:topicName','/gripper/status'),('PublishGripperStatus.inputs:messagePackage','std_msgs'),
        ('PublishGripperStatus.inputs:messageSubfolder','msg'),('PublishGripperStatus.inputs:messageName','String'),('PublishGripperStatus.inputs:data','')]
    connections=[('Time.outputs:simulationTime','Clock.inputs:timeStamp'),('Time.outputs:simulationTime','JointState.inputs:timeStamp')]
    for node in ['Clock','JointState','JointCommand','PublishGripperStatus']:
        connections.extend([('Tick.outputs:tick',node+'.inputs:execIn'),('Context.outputs:context',node+'.inputs:context')])
    connections.extend([('Tick.outputs:tick','Arm.inputs:execIn'),('JointCommand.outputs:jointNames','Arm.inputs:jointNames'),
        ('JointCommand.outputs:positionCommand','Arm.inputs:positionCommand'),('JointCommand.outputs:velocityCommand','Arm.inputs:velocityCommand'),('JointCommand.outputs:effortCommand','Arm.inputs:effortCommand')])
    for name,topic in [('SubscribeToolIntent','/tool/intent'),('SubscribeGripperCaptureReference','/gripper/capture_reference')]:
        nodes.append((name,'isaacsim.ros2.bridge.ROS2Subscriber'))
        values.extend([(name+'.inputs:topicName',topic),(name+'.inputs:messagePackage','std_msgs'),(name+'.inputs:messageSubfolder','msg'),(name+'.inputs:messageName','String')])
        connections.extend([('Tick.outputs:tick',name+'.inputs:execIn'),('Context.outputs:context',name+'.inputs:context')])
    nodes.append(('ToolIntentGeneration','omni.graph.action.Counter'))
    connections.append(('SubscribeToolIntent.outputs:execOut','ToolIntentGeneration.inputs:execIn'))
    return nodes,values,connections


def build_action_graph(*, configuration, namespace):
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from isaac_runtime.robot_binding import RobotBinding
    enable_extension('isaacsim.ros2.bridge');enable_extension('isaacsim.core.nodes')
    path=configuration['cell']['isaac']['action_graph_path']
    stage=omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(path):stage.RemovePrim(path)
    nodes,values,connections=graph_declaration(RobotBinding.from_runtime(configuration))
    og.Controller.edit(dict(graph_path=path,evaluator_name='execution'),{
        og.Controller.Keys.CREATE_NODES:nodes,og.Controller.Keys.SET_VALUES:values,og.Controller.Keys.CONNECT:connections})
