"""Resolve and validate an assembly's Robot and Tool without compiling a Cell mission."""
from .runtime import read, resolve


def load_assembly_model(robot_id, tool_id):
    """Load static assembly owners; this does not register a deployment or execution."""
    from .configured_supply import radial_tool_model

    config = {}
    references = {
        'robot': f'package://arm_cell_motion/config/robots/{robot_id}.yaml',
        'tool': f'package://arm_cell_motion/config/tools/{tool_id}.yaml',
        'model_configuration': f'package://arm_cell_motion/config/models/{robot_id}_{tool_id}.yaml',
        'motion_configuration': 'package://arm_cell_motion/config/planning.yaml',
    }
    for key, reference in references.items():
        owner = resolve(reference, '.')
        config[key] = read(owner)
        fields = {'robot': ('home', 'joint_limits'), 'tool': (),
                  'model_configuration': ('urdf', 'mesh_root', 'semantic'),
                  'motion_configuration': tuple(config[key])}[key]
        for field in fields:
            config[key][field] = str(resolve(config[key][field], owner.parent))
    model = config['model_configuration']
    if any(model[f'{key}_id'] != config[key][f'{key}_id'] for key in ('robot', 'tool')):
        raise ValueError('Assembly owner identity mismatch')
    config['home_joint_positions'] = read(config['robot']['home'])['home_joint_positions']
    return radial_tool_model(config)
