"""Launch output timestamps share one observation-time basis per session."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch.logging import launch_config


def _load_formatter(source, name):
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ConsoleOutputFormat


def test_formatters_share_session_elapsed_time_and_preserve_console_format(
    tmp_path, monkeypatch
):
    source = Path(__file__).resolve().parents[1] / "launch/console_output_format.py"
    formatter_a = _load_formatter(source, "console_output_format_a")
    formatter_b = _load_formatter(source, "console_output_format_b")
    observed_ns = iter((1_000, 1_010, 1_100, 2_000))
    monkeypatch.setattr("time.monotonic_ns", lambda: next(observed_ns))
    monkeypatch.setattr(launch_config, "_log_dir", str(tmp_path))
    monkeypatch.setattr(launch_config, "file_handlers", {})

    process_a = SimpleNamespace(process_description=SimpleNamespace(final_name="a"))
    process_b = SimpleNamespace(process_description=SimpleNamespace(final_name="b"))
    output_a = formatter_a("{line}")
    output_b = formatter_b("{line}")

    assert output_a.format(line="first", this=process_a) == "[0.000s] first"
    assert output_b.format(line="second", this=process_b) == "[0.000s] second"

    saved = (tmp_path / "ros_output_nanoseconds.log").read_text()
    assert "launch output observation time" in saved.splitlines()[0]
    assert "ROS event time" in saved.splitlines()[0]
    assert "[0.000000100s] [a] first" in saved
    assert "[0.000001000s] [b] second" in saved


def test_formatter_preserves_flattened_ompl_and_motion_lines(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[1] / "launch/console_output_format.py"
    formatter_type = _load_formatter(source, "console_output_format_ompl")
    observed_ns = iter((1_000, 1_010, 1_020, 1_030, 1_040))
    monkeypatch.setattr("time.monotonic_ns", lambda: next(observed_ns))
    monkeypatch.setattr(launch_config, "_log_dir", str(tmp_path))
    monkeypatch.setattr(launch_config, "file_handlers", {})

    move_group = SimpleNamespace(
        process_description=SimpleNamespace(final_name="move_group")
    )
    formatter = formatter_type("{line}")
    error_line = "[ERROR] [ompl] ./src/ompl/RRTConnect.cpp:265 - planner failed"
    warning_line = "[WARN] [ompl] ./src/ompl/ParallelPlan.cpp:138 - no solution"
    motion_result = "[ERROR] [moveit] Motion planning failed; context: [ompl]"

    assert formatter.format(line=error_line, this=move_group).endswith(error_line)
    assert formatter.format(line=warning_line, this=move_group).endswith(warning_line)
    assert formatter.format(line=motion_result, this=move_group).endswith(motion_result)

    saved = (tmp_path / "ros_output_nanoseconds.log").read_text()
    assert error_line in saved
    assert warning_line in saved
    assert motion_result in saved


def test_ompl_raw_ros_logs_remain_on_console_if_file_logging_fails(
    tmp_path, monkeypatch
):
    source = Path(__file__).resolve().parents[1] / "launch/console_output_format.py"
    formatter_type = _load_formatter(source, "console_output_format_ompl_file_error")
    observed_ns = iter((1_000, 1_010))
    monkeypatch.setattr("time.monotonic_ns", lambda: next(observed_ns))
    monkeypatch.setattr(launch_config, "_log_dir", str(tmp_path))
    monkeypatch.setattr(launch_config, "file_handlers", {})

    def fail_open(*args, **kwargs):
        raise OSError("file log unavailable")

    monkeypatch.setattr(Path, "open", fail_open)
    move_group = SimpleNamespace(
        process_description=SimpleNamespace(final_name="move_group")
    )
    formatter = formatter_type("{line}")
    error_line = "[ERROR] [ompl] ./src/ompl/RRTConnect.cpp:265 - planner failed"

    assert formatter.format(line=error_line, this=move_group).endswith(error_line)
