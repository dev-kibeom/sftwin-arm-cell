# Offline D455 assets

Before the first offline run, collect NVIDIA's D455 asset and its USD, MDL,
material and texture dependencies while connected to the internet:

```bash
~/isaac_sim/python.sh infra/isaac_sim/scripts/prepare_d455_assets.py
```

Run from the checkout, then start `infra/isaac_sim/scripts/isaac_arm_cell.sh`
as usual. The scene uses `.local_artifacts/isaac/d455/rsd455.usd` without
checking the online asset root. Missing local assets fail immediately with
preparation instructions. The collector publishes the directory only after
successful collection and refuses to overwrite an existing installation.
Move the directory aside before refreshing it.

To use an independently collected D455 asset, set `SFTWIN_D455_ASSET` to its
local USD path. Its referenced USD files, materials and textures must also
be local. Copy the entire collected directory when moving machines.

Downloaded NVIDIA assets remain under their original license and are not
committed to this repository. This change removes the scene's remote D455
dependency; Kit may still attempt optional update or extension registry checks.
