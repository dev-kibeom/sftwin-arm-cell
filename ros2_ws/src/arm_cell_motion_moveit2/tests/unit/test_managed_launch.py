from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).parents[1]))

from acceptance.managed_launch import ManagedLaunch  # noqa: E402


MARKER = "wu13-owned-launch-regression"


def process_count(marker):
    count = 0
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            command = (entry / "cmdline").read_bytes().decode(errors="replace")
        except (FileNotFoundError, PermissionError):
            continue
        if marker in command:
            count += 1
    return count


def owned_tree_command():
    child = "import time; time.sleep(30) # " + MARKER
    parent = (
        "import subprocess,sys,time; "
        f"subprocess.Popen([sys.executable, '-c', {child!r}]); time.sleep(30) # {MARKER}"
    )
    return [sys.executable, "-c", parent]


def test_managed_launch_tears_down_complete_tree_and_is_repeatable(tmp_path):
    baseline = process_count(MARKER)
    for _ in range(2):
        with ManagedLaunch(owned_tree_command(), cwd=tmp_path):
            deadline = time.monotonic() + 2.0
            while process_count(MARKER) < baseline + 2 and time.monotonic() < deadline:
                time.sleep(0.05)
            assert process_count(MARKER) >= baseline + 2
        assert process_count(MARKER) == baseline
