"""USD Preview Surface material authoring and binding."""


def _pxr_modules():
    from pxr import Gf, Sdf, UsdShade

    return Gf, Sdf, UsdShade


def create_preview_material(
    stage,
    cell_root,
    name,
    color,
    roughness=0.45,
    metallic=0.0,
    opacity=1.0,
    *,
    gf=None,
    sdf=None,
    usd_shade=None,
):
    """Author one UsdPreviewSurface material below the cell Materials scope."""
    if gf is None or sdf is None or usd_shade is None:
        default_gf, default_sdf, default_usd_shade = _pxr_modules()
        gf = gf or default_gf
        sdf = sdf or default_sdf
        usd_shade = usd_shade or default_usd_shade

    mat_path = f"{cell_root}/Materials/{name}"
    shader_path = f"{mat_path}/Shader"

    mat = usd_shade.Material.Define(stage, mat_path)
    shader = usd_shade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", sdf.ValueTypeNames.Color3f).Set(
        gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("roughness", sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("opacity", sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind_material(stage, prim_path, material, *, usd_shade=None):
    """Bind an authored material if the target prim exists."""
    if usd_shade is None:
        _, _, usd_shade = _pxr_modules()
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        usd_shade.MaterialBindingAPI(prim).Bind(material)
