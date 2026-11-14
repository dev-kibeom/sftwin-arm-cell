"""Low-level USD xform, metadata, and custom-attribute authoring."""


def _pxr_modules():
    from pxr import Gf, Sdf, UsdGeom

    return Gf, Sdf, UsdGeom


def ensure_xform(stage, path, *, usd_geom=None):
    """Return an existing prim or define an Xform at ``path``."""
    if usd_geom is None:
        _, _, usd_geom = _pxr_modules()
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return usd_geom.Xform.Define(stage, path).GetPrim()
    return prim


def set_custom_data(stage, path, key, value):
    """Set metadata only when the authored prim exists."""
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        prim.SetCustomDataByKey(key, value)


def set_vec3_attribute(stage, path, name, value, *, gf=None, sdf=None, warn=print):
    """Author a custom Double3 attribute, preserving the builder warning contract."""
    if gf is None or sdf is None:
        default_gf, default_sdf, _ = _pxr_modules()
        gf = gf or default_gf
        sdf = sdf or default_sdf

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        warn(f">>> [WARN] Cannot create vec3 attribute; invalid prim: {path}")
        return None

    attr = prim.GetAttribute(name)
    if not attr.IsValid():
        attr = prim.CreateAttribute(name, sdf.ValueTypeNames.Double3, custom=True)

    attr.Set(gf.Vec3d(float(value[0]), float(value[1]), float(value[2])))
    return attr
