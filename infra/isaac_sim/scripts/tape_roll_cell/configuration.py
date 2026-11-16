"""Export an isolated experiment; never advertise an operational Cell Profile."""
import argparse
import json
from pathlib import Path
import yaml
from arm_cell_bringup.model_experiment import load_model_experiment
from arm_cell_bringup.model_validation import joint_pose, validate_assembly_configuration, planning_limits
from tape_roll_cell.material import TapeRoll


def compose(profile, experiment_path):
    config = load_model_experiment(profile)
    experiment = yaml.safe_load(Path(experiment_path).read_text())
    TapeRoll(**experiment['material'])
    tool_id = config['tool']['tool_id']
    teaching = experiment.pop('teaching')[tool_id]
    _, joints, arm = validate_assembly_configuration(config)
    overrides = planning_limits(config['robot']['joint_limits'], arm, joints)
    for name, pose in teaching.items():
        joint_pose(pose, name, arm, joints, overrides)
    experiment['teaching'] = teaching
    config['experiment_drive_stiffness_scale'] = 100.0
    config['cell'] = dict(isaac={'scene_builder': 'tape_roll_cell.scene:build_scene'}, tape_roll=experiment)
    # Reuse selected Tool runtime construction; only this experiment opts in.
    config['tool']['runtime']['implementation'] = 'hand_e_c10_physical'
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--experiment', default=str(Path(__file__).with_name('experiment.yaml')))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    Path(args.output).write_text(json.dumps(compose(args.profile,args.experiment),indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
