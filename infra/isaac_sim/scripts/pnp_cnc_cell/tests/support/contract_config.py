"""Load the M0609 RGB-D acceptance contract without depending on pytest."""

from pathlib import Path

import yaml


def acceptance_config():
    path = Path(__file__).resolve().parents[1] / "acceptance" / "acceptance.yaml"
    return yaml.safe_load(path.read_text())
