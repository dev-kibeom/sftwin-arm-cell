"""Pytest path setup for M0609 cell-owned verification code."""

import sys
from pathlib import Path


TEST_ROOT = Path(__file__).resolve().parent
CELL_MODULE_ROOT = TEST_ROOT.parent
if str(CELL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CELL_MODULE_ROOT))

sys.path.insert(0, str(CELL_MODULE_ROOT.parent))

for directory in ("support", "tooling", "acceptance"):
    path = str(TEST_ROOT / directory)
    if path not in sys.path:
        sys.path.insert(0, path)

# Integration fixtures export through the producer API. Runtime consumers only
# receive the resulting file and do not import the producer implementation.
BRINGUP_SOURCE = Path(__file__).resolve().parents[5] / 'ros2_ws/src/bringup'
if str(BRINGUP_SOURCE) not in sys.path:
    sys.path.insert(0, str(BRINGUP_SOURCE))

import pytest


@pytest.fixture(scope='session')
def exported_configuration(tmp_path_factory):
    from arm_cell_bringup.runtime import export_runtime_configuration
    destination = tmp_path_factory.mktemp('runtime-contract') / 'operational.json'
    return export_runtime_configuration(BRINGUP_SOURCE / 'config/operational_profile.yaml', destination)


@pytest.fixture(autouse=True)
def _runtime_configuration_file(monkeypatch, exported_configuration):
    monkeypatch.setenv('ARM_CELL_RUNTIME_CONFIG', str(exported_configuration))
