"""Format and persist Launch-observed output with elapsed-time prefixes."""

import logging
from pathlib import Path
import re
import sys
import time

from launch.logging import get_logger, launch_config
from launch_ros.actions import Node


class _LaunchOutput:
    """One clock and output file for the nodes in a launch session."""

    def __init__(self):
        self.started_at_ns = time.monotonic_ns()
        self.directory = None
        self.stream = None
        self.failed = False

    def select_directory(self, directory):
        if directory != self.directory:
            if self.stream is not None:
                try:
                    self.stream.close()
                except OSError as error:
                    self.report_failure(error)
            self.directory = directory
            self.stream = None
            self.failed = False

    def report_failure(self, error):
        self.failed = True
        message = f"ROS output file incomplete / {error}"
        try:
            get_logger("arm_cell_bringup").warning(message)
        except OSError:
            # The launch log directory can also be unavailable.
            try:
                print(f"[WARN] [arm_cell_bringup] {message}", file=sys.stderr)
            except (OSError, ValueError):
                pass

    def write(self, line, elapsed_ns, name):
        if self.failed:
            return
        try:
            if self.stream is None:
                directory = Path(self.directory or launch_config.log_dir)
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / "ros_output_nanoseconds.log"
                self.stream = path.open("a", encoding="utf-8", buffering=1)
                self.stream.write(
                    "# clock=monotonic_ns; time_basis=launch output observation time; "
                    "origin=first ARM Cell output initialization in this Launch session; "
                    "ROS event time=false\n"
                )

            seconds, nanoseconds = divmod(elapsed_ns, 1_000_000_000)
            self.stream.write(f"[{seconds}.{nanoseconds:09d}s] [{name}] {line}\n")
        except OSError as error:
            self.report_failure(error)
            if self.stream is not None:
                try:
                    self.stream.close()
                except OSError:
                    pass
                self.stream = None


def session_output():
    # Launch resets its file handlers when it starts a new logging session.
    if getattr(launch_config, "_arm_cell_handlers", None) is not launch_config.file_handlers:
        launch_config._arm_cell_handlers = launch_config.file_handlers
        launch_config._arm_cell_output = _LaunchOutput()
    return launch_config._arm_cell_output


# rcutils supplies the emitting file even when rclpy uses the application's logger.
# Host and Docker compile from ros2_ws/src. Installed Python entry points report
# their install path when colcon builds without --symlink-install.
_SOURCE = re.compile(
    r"(?:^|/)ros2_ws/src/(?:bringup|gateway|motion|orchestration|plc_adapter|"
    r"safety_fsm|sim_adapter|task_execution|vda_adapter|vision)/"
    r"(?:src|include|scripts|arm_cell_[^/]+)/"
    r"|/(?:lib|site-packages|dist-packages)/arm_cell_[a-z0-9_]+/(?:[a-zA-Z0-9_]+/)*[^/]+\.py$"
)
_NATIVE_FORMAT = "arm_cell_ros\t{file_name}\t[{severity}] [{name}] {message}"


# Launch calls logger.info() for every formatted line. A private sentinel lets
# its standard handlers omit rejected records entirely, including blank rows.
_SKIPPED = object()


def _visible(record):
    return record.msg is not _SKIPPED


class _OperatorFormat(str):
    def format(self, *, line, this):
        output = session_output()
        name = this.process_description.final_name
        # Launch can flush a final chunk after closing its line buffer.
        lines = []
        for part in line.splitlines():
            tag, separator, rest = part.partition("\t")
            if tag != "arm_cell_ros" or not separator:
                continue
            source, separator, payload = rest.partition("\t")
            if separator and _SOURCE.search(source):
                elapsed = max(0, time.monotonic_ns() - output.started_at_ns)
                output.write(payload, elapsed, name)
                lines.append(f"[{elapsed / 1_000_000_000:.3f}s] {payload}")
        return "\n".join(lines) if lines else _SKIPPED


class OperatorLogNode(Node):
    """Route child output through the operator log before any launch destination."""

    def __init__(self, *, ros_arguments=(), **kwargs):
        super().__init__(
            **kwargs,
            output="screen",
            output_format=_OperatorFormat("{line}"),
            additional_env={
                "RCUTILS_CONSOLE_OUTPUT_FORMAT": _NATIVE_FORMAT,
                "RCUTILS_COLORIZED_OUTPUT": "0",
            },
            ros_arguments=[*ros_arguments, "--disable-external-lib-logs", "--disable-rosout-logs"],
        )

    def execute(self, context):
        actions = super().execute(context)
        # Node.execute schedules the process. Its I/O callbacks run after this returns.
        name = self.process_description.final_name
        session_output()
        for stream in ("stdout", "stderr"):
            logging.getLogger(f"{name}-{stream}").addFilter(_visible)
        return actions


if __name__ == "__main__":
    # The GUI starts its optional PLC adapter separately from the cell launch.
    from launch import LaunchDescription, LaunchService

    node = OperatorLogNode(
        executable=sys.argv[1], prefix=sys.executable, ros_arguments=sys.argv[2:],
        # Finish escalation within the GUI's five-second termination deadline.
        sigterm_timeout="2", sigkill_timeout="1",
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([node]))
    status = service.run()
    sys.exit(status or (node.return_code if node.return_code is not None else 1))
