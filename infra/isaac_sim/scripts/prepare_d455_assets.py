"""Collect the D455 USD, materials and textures once while online.

Run using Isaac Sim's python.sh. Runtime never invokes this downloader.
"""
import asyncio
import os
from pathlib import Path
import shutil
import tempfile

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.kit.usd.collect")
    from omni.kit.usd.collect import Collector, CollectorFailureOptions

    root = Path(os.environ.get("SFTWIN_PROJECT_ROOT") or Path(__file__).resolve().parents[3])
    target = root / ".local_artifacts/isaac/d455"
    source = (
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError(f"Destination already exists: {target}; move it aside before refreshing")
    with tempfile.TemporaryDirectory(dir=target.parent) as temporary:
        collector = Collector(
            source, temporary, flat_collection=True,
            failure_options=(CollectorFailureOptions.EXTERNAL_USD_REFERENCES |
                             CollectorFailureOptions.OTHER_EXTERNAL_REFERENCES),
        )
        try:
            task = asyncio.ensure_future(collector.collect())
            while not task.done():
                app.update()
            success, collected_path = task.result()
            if not success:
                raise RuntimeError(f"D455 collection failed: {collected_path}")
            if not (Path(temporary) / "rsd455.usd").is_file():
                raise RuntimeError("Collector did not produce rsd455.usd")
            shutil.move(temporary, target)
        finally:
            collector.destroy()
    print(f"Prepared offline D455 assets: {target}", flush=True)
finally:
    app.close()
