"""Static USD API/topology check; this does not simulate contact or friction.

Can also run directly with Isaac's Python and USD/PhysX extensions on the
module/library paths. ARM_CELL_PHYSX_PLUGIN may name its plugInfo.json.
"""
from pathlib import Path
import os


def verify_contact_topology():
    from pxr import Plug
    plugin = os.environ.get('ARM_CELL_PHYSX_PLUGIN')
    if plugin:
        Plug.Registry().RegisterPlugins(plugin)
    from pxr import Usd, UsdPhysics, PhysxSchema
    root = Path(__file__).resolve().parents[4] / 'assets/usd'
    for model in ('m0609', 'rb5_850'):
        stage = Usd.Stage.Open(str(root / (model + '_robotiq_2f85_generated.usd')))
        assert stage
        for side in ('left', 'right'):
            name = 'gripper_robotiq_85_' + side + '_finger_tip_link'
            bodies = [p for p in stage.Traverse() if p.GetName() == name
                      and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            assert len(bodies) == 1, (model, name)
            body = bodies[0]
            colliders = [p for p in Usd.PrimRange(body, Usd.TraverseInstanceProxies())
                         if p.HasAPI(UsdPhysics.CollisionAPI)]
            assert colliders, (model, name)
            assert all(p.GetPath().HasPrefix(body.GetPath()) for p in colliders)
            assert PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.).Get() == 0.
        print(model + ': fingertip rigid bodies, child colliders, ContactReportAPI verified')


def test_selected_tool_contact_topology():
    import pytest
    pytest.importorskip('pxr.Usd')
    verify_contact_topology()


if __name__ == '__main__':
    verify_contact_topology()
