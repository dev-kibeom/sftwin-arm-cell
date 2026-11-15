"""Per-step normal reaction on opposing fingertips, in newtons."""
import math


def finger_reactions(contacts, dt, *, target_path=None, fingertip_paths=None, require_bilateral=True):
    if contacts is None or not math.isfinite(dt) or dt <= 0:
        return None
    targets = {}
    try:
        for contact in contacts:
            if contact['lost']:
                continue
            for actor, other in (('actor0', 'actor1'), ('actor1', 'actor0')):
                path = contact[actor]
                for side in ('left', 'right'):
                    suffix = '/gripper_robotiq_85_' + side + '_finger_tip_link'
                    if not path.endswith(suffix):
                        continue
                    if fingertip_paths is not None and path != fingertip_paths[0 if side == 'left' else 1]:
                        continue
                    if target_path is not None and contact[other] != target_path:
                        continue
                    root = path[:-len(suffix)]
                    if contact[other].startswith(root + '/'):
                        continue
                    forces = targets.setdefault((root, contact[other]), [0., 0.])
                    for point in contact['points']:
                        impulse, normal = point['impulse'], point['normal']
                        if len(impulse) != 3 or len(normal) != 3 or not all(
                                math.isfinite(v) for v in (*impulse, *normal)):
                            return None
                        reaction = abs(sum(a*b for a, b in zip(impulse, normal))) / dt
                        index = 0 if side == 'left' else 1
                        forces[index] += reaction
                        if not math.isfinite(forces[index]):
                            return None
        # Require both fingers on the same opposing body. Do not combine bodies.
        return tuple(max((f for f in targets.values() if not require_bilateral or min(f) > 0), key=lambda f: min(f) if require_bilateral else sum(f), default=(0., 0.)))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def reaction_diagnostics(contacts, dt):
    """Preserve unpaired loads per robot/body; never used as qualification."""
    if contacts is None or not math.isfinite(dt) or dt <= 0:
        return None
    pairs = set()
    for contact in contacts:
        for actor, other in (('actor0', 'actor1'), ('actor1', 'actor0')):
            for side in ('left', 'right'):
                suffix = '/gripper_robotiq_85_' + side + '_finger_tip_link'
                if contact[actor].endswith(suffix):
                    root = contact[actor][:-len(suffix)]
                    if not contact[other].startswith(root + '/'):
                        pairs.add((root, contact[other]))
    return [dict(robot_path=root, target_path=target, finger_reactions_n=finger_reactions(
        contacts, dt, target_path=target, fingertip_paths=tuple(
            root + '/gripper_robotiq_85_' + side + '_finger_tip_link' for side in ('left', 'right')),
        require_bilateral=False)) for root, target in sorted(pairs)]
