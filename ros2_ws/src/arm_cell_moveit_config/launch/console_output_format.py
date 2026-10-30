"""Format and persist Launch-observed output with elapsed-time prefixes."""

import time
from pathlib import Path


class ConsoleOutputFormat(str):
    """Prefix console output and persist a nanosecond-timed copy for analysis."""

    def __new__(cls, template):
        instance = super().__new__(cls, template)
        instance._started_at_ns = time.monotonic_ns()
        instance._file_log = None
        from launch.logging import launch_config

        session = launch_config.file_handlers
        if getattr(launch_config, "_console_output_session", None) is not session:
            launch_config._console_output_session = session
            launch_config._console_output_started_at_ns = instance._started_at_ns
            launch_config._console_output_header_written = False
        instance._session_started_at_ns = launch_config._console_output_started_at_ns
        return instance

    def format(self, **kwargs):
        line = super().format(**kwargs)
        observed_at_ns = time.monotonic_ns()
        console_elapsed_ns = max(0, observed_at_ns - self._started_at_ns)
        session_elapsed_ns = max(0, observed_at_ns - self._session_started_at_ns)
        self._write_nanosecond_file_log(line, session_elapsed_ns, kwargs.get("this"))
        elapsed = console_elapsed_ns / 1_000_000_000
        return f"[{elapsed:.3f}s] {line}"

    def _write_nanosecond_file_log(self, line, elapsed_ns, process):
        """Write output with elapsed time from the shared Launch session origin."""
        if process is None or self._file_log is False:
            return

        try:
            if self._file_log is None:
                from launch.logging import launch_config

                log_path = Path(launch_config.log_dir) / "ros_output_nanoseconds.log"
                self._file_log = log_path.open("a", encoding="utf-8", buffering=1)
                if not launch_config._console_output_header_written:
                    self._file_log.write(
                        "# clock=monotonic_ns; time_basis=launch output observation "
                        "time; origin=first ConsoleOutputFormat initialization in "
                        "this Launch session; ROS event time=false\n"
                    )
                    launch_config._console_output_header_written = True

            process_description = getattr(process, "process_description", None)
            process_name = getattr(process_description, "final_name", None) or "process"
            seconds, nanoseconds = divmod(elapsed_ns, 1_000_000_000)
            self._file_log.write(
                f"[{seconds}.{nanoseconds:09d}s] [{process_name}] {line}\n"
            )
        except OSError:
            # Log persistence must not interfere with the existing terminal output.
            if self._file_log is not None:
                try:
                    self._file_log.close()
                except OSError:
                    pass
            self._file_log = False
