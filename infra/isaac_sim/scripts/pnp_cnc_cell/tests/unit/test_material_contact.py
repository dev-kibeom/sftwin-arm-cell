"""Verify the material creation path with real USD; no physics simulation."""

def verify_material_contact():
    from pxr import Usd, UsdShade, UsdPhysics, PhysxSchema
    from scene_builder.material_supply import MaterialSupplyProfile
    from scene_builder.material_supply_adapter import IsaacMaterialSupplyAdapter
    stage=Usd.Stage.CreateInMemory()
    adapter=IsaacMaterialSupplyAdapter.__new__(IsaacMaterialSupplyAdapter)
    adapter.stage=stage
    adapter._delivery_profiles={}
    adapter.profile=MaterialSupplyProfile('TestCube','box',(0.07,0.07,0.08),0.4,
        (-0.1,0.1),(-0.1,0.1),0.3,0.5,((0.,0.),)*3,
        static_friction=1.,dynamic_friction=0.8)
    path=adapter._create_part('test',0,(0.,0.,0.,0.,0.,0.),part_root_path='/World/parts')
    collision=stage.GetPrimAtPath(path+'/collision')
    material,_=UsdShade.MaterialBindingAPI(collision).ComputeBoundMaterial('physics')
    assert material
    surface=UsdPhysics.MaterialAPI(material.GetPrim())
    assert surface.GetStaticFrictionAttr().Get()==1.
    assert abs(surface.GetDynamicFrictionAttr().Get()-0.8)<1e-6
    assert PhysxSchema.PhysxMaterialAPI(material.GetPrim()).GetFrictionCombineModeAttr().Get()=='average'
    print('Material collider binding and explicit friction verified')


def test_material_contact_binding():
    import pytest
    pytest.importorskip('pxr.Usd')
    verify_material_contact()
