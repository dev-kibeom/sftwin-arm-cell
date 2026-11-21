"""Format and persist Launch-observed output with elapsed-time prefixes."""

from pathlib import Path
import sys
import time

from launch.logging import get_logger, launch_config


class _LaunchOutput:
    """One clock and output file for all formatter imports in a Launch session."""

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

    def write(self, line, elapsed_ns, process):
        if self.failed or process is None:
            return
        try:
            if self.stream is None:
                directory = Path(self.directory or launch_config.log_dir)
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / "ros_output_nanoseconds.log"
                self.stream = path.open("a", encoding="utf-8", buffering=1)
                self.stream.write(
                    "# clock=monotonic_ns; time_basis=launch output observation time; "
                    "origin=first ConsoleOutputFormat initialization in this Launch session; "
                    "ROS event time=false\n"
                )

            description = getattr(process, "process_description", None)
            name = getattr(description, "final_name", None) or "process"
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


class ConsoleOutputFormat(str):
    """Prefix flattened ROS output without modifying backend fields."""

    def __new__(cls, template):
        instance = super().__new__(cls, template)
        session = launch_config.file_handlers
        if getattr(launch_config, "_console_output_session", None) is not session:
            launch_config._console_output_session = session
            launch_config._arm_cell_output = _LaunchOutput()
        instance._output = launch_config._arm_cell_output
        return instance

    def set_record_directory(self, directory):
        self._output.select_directory(directory)

    def format(self, **kwargs):
        line = super().format(**kwargs)
        elapsed_ns = max(0, time.monotonic_ns() - self._output.started_at_ns)
        self._output.write(line, elapsed_ns, kwargs.get("this"))
        return f"[{elapsed_ns / 1_000_000_000:.3f}s] {line}"
