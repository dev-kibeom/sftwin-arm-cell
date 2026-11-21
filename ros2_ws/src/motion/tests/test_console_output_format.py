"""Verify which ROS output reaches the operator on screen and on disk."""

import os
import subprocess
import sys
from textwrap import dedent
from types import SimpleNamespace

from launch.logging import launch_config

from arm_cell_motion.launch_output import _OperatorFormat, session_output


def _format(process, source, message):
    node = SimpleNamespace(process_description=SimpleNamespace(final_name=process))
    return _OperatorFormat("{line}").format(line=f"arm_cell_ros\t{source}\t{message}", this=node)


def test_nested_nodes_share_clock_and_selected_run_file(tmp_path, monkeypatch):
    monkeypatch.setattr(launch_config, "file_handlers", {})
    ticks = iter((1_000_000_000, 4_000_000_000, 5_000_000_000))
    monkeypatch.setattr("time.monotonic_ns", lambda: next(ticks))
    output = session_output()
    output.select_directory(tmp_path)
    for process, source, message, elapsed in (
        ("motion", "/workspace/ros2_ws/src/motion/src/motion_core.cpp", "first", 3),
        ("mission", "/opt/arm/lib/arm_cell_orchestration/mission.py", "second", 4),
    ):
        assert _format(process, source, message) == f"[{elapsed}.000s] {message}"
    saved = (tmp_path / "ros_output_nanoseconds.log").read_text()
    assert saved.count("# clock=") == 1
    assert "[3.000000000s] [motion] first" in saved
    assert "[4.000000000s] [mission] second" in saved


def test_project_error_survives_file_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(launch_config, "_log_dir", str(tmp_path))
    monkeypatch.setattr(launch_config, "file_handlers", {})
    blocked = tmp_path / "not-a-directory"
    blocked.touch()
    session_output().select_directory(blocked)
    for _ in range(2):
        message = "[ERROR] [motion] Planning failed"
        line = _format("motion", "/workspace/ros2_ws/src/motion/src/executor.cpp", message)
        assert line.endswith(message)
    assert (tmp_path / "launch.log").read_text().count("ROS output file incomplete") == 1


def test_real_launch_rejects_library_and_unframed_output(tmp_path):
    # Run a child through the actual ROS launch I/O handlers, including their
    # file destinations. Logging alone needs no ROS graph or simulator.
    (tmp_path / "dependency.py").write_text(dedent("""\
        def emit(logger):
            logger.debug("LIBRARY_DEBUG")
            logger.info("LIBRARY_INFO")
            logger.warning("LIBRARY_GOAL_UUID")
            logger.error("LIBRARY_ERROR")
            logger.fatal("LIBRARY_FATAL")
        """))
    project = tmp_path / "ros2_ws/src/orchestration/scripts/probe.py"
    project.parent.mkdir(parents=True)
    project.write_text(dedent("""\
        #!/usr/bin/python3
        import sys
        from rclpy.logging import get_logger, LoggingSeverity
        from dependency import emit
        logger = get_logger("orchestration_node")
        logger.set_level(LoggingSeverity.DEBUG)
        assert "--disable-external-lib-logs" in sys.argv
        assert "--disable-rosout-logs" in sys.argv
        emit(logger)
        print("LIBRARY_STDOUT", flush=True)
        print("LIBRARY_STDERR", file=sys.stderr, flush=True)
        logger.error("Mission failed / planning unavailable")
        sys.exit(17)
        """))
    project.chmod(0o755)
    result = subprocess.run(
        [sys.executable, "-m", "arm_cell_motion.launch_output", str(project)],
        env={
            **os.environ,
            "PYTHONPATH": os.pathsep.join((str(tmp_path), os.getenv("PYTHONPATH", ""))),
            "ROS_LOG_DIR": str(tmp_path / "logs"),
            "OVERRIDE_LAUNCH_PROCESS_OUTPUT": "full",
        },
        capture_output=True, text=True, timeout=20,
    )
    console = result.stdout + result.stderr
    assert result.returncode == 17, console
    expected = "[ERROR] [orchestration_node] Mission failed / planning unavailable"
    assert expected in console
    logs = list((tmp_path / "logs").rglob("*.log"))
    saved = next(path for path in logs if path.name == "ros_output_nanoseconds.log")
    assert expected in saved.read_text()
    for output in [console, *(log.read_text() for log in logs)]:
        assert "LIBRARY_" not in output
        assert "arm_cell_ros\t" not in output
