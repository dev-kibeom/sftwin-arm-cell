"""Linux host preflight for the ARM Cell ROS entry point."""
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import signal
import sys
import time


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    start: int
    argv: tuple
    cwd: Path | None


def snapshot(proc_root=Path('/proc')):
    processes = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            argv = tuple(part.decode(errors='replace') for part in
                         (entry / 'cmdline').read_bytes().split(b'\0') if part)
            try:
                cwd = (entry / 'cwd').resolve(strict=True)
            except PermissionError:
                # Same-UID protected processes can expose cmdline/stat but deny
                # cwd. Retain their identity; classification decides relevance.
                cwd = None
            processes[int(entry.name)] = Process(
                int(entry.name), int(fields[1]), int(fields[19]), argv,
                cwd)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as exc:
            raise RuntimeError(f'프로세스 확인 권한 없음: {entry.name}') from exc
    return processes


def is_bringup(process):
    return any(Path(arg).name == 'ros2' and
               process.argv[index + 1:index + 4] ==
               ('launch', 'arm_cell_bringup', 'arm_cell.launch.py')
               for index, arg in enumerate(process.argv))


def classify(processes, root, current_pid):
    owned = {pid: p for pid, p in processes.items()
             if pid != current_pid and is_bringup(p) and p.cwd == root}
    while True:
        children = {pid: p for pid, p in processes.items()
                    if p.parent in owned and pid not in owned}
        if not children:
            break
        owned.update(children)
    # Include Python entry points and generic ROS binaries orphaned by launch.
    # Unknown ownership blocks startup; it never authorizes a signal.
    ros_names = {'move_group', 'robot_state_publisher', 'static_transform_publisher', 'rviz2'}
    def installed_component(p):
        if not p.argv:
            return False
        executable = p.argv[0]
        if Path(executable).name.startswith('python') and len(p.argv) > 1:
            executable = p.argv[1]
        return (executable.startswith('/') and
                '/ros2_ws/install/arm_cell_' in executable)

    foreign = [pid for pid, p in processes.items() if pid not in owned and
               pid != current_pid and (is_bringup(p) or
               installed_component(p) or
               (p.cwd in (root, None) and p.argv and Path(p.argv[0]).name in ros_names))]
    if foreign:
        raise RuntimeError(f'소유 관계를 확인할 수 없는 ARM Cell 프로세스: {foreign}; '
                           '해당 실행을 종료한 뒤 다시 시작하세요.')
    return owned


def stop_processes(processes, timeouts=(15, 5, 5)):
    remaining = dict(processes)
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        current = snapshot()
        remaining = {pid: p for pid, p in remaining.items()
                     if pid in current and current[pid].start == p.start}
        for pid, process in remaining.items():
            # pidfd binds the signal to this process even if its PID is recycled.
            try:
                fd = os.pidfd_open(pid)
                try:
                    check = snapshot().get(pid)
                    if check and check.start == process.start:
                        signal.pidfd_send_signal(fd, sig)
                finally:
                    os.close(fd)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while remaining:
            current = snapshot()
            remaining = {pid: p for pid, p in remaining.items()
                         if pid in current and current[pid].start == p.start}
            if not remaining or time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        if not remaining:
            return
    raise RuntimeError(f'이전 ROS 프로세스 종료 실패: {list(remaining)}')


def main():
    root = Path(__file__).resolve().parents[2]
    # Hold through exec and all launch descendants; concurrent starts fail closed.
    lock_path = Path('/tmp') / f'arm-cell-ros-{os.getuid()}.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('다른 관리 중인 ARM Cell bringup이 실행 중입니다. 먼저 종료하세요.')
    owned = classify(snapshot(), root, os.getpid())
    if owned:
        print(f'이전 ARM Cell bringup 종료: {sorted(owned)}', flush=True)
        stop_processes(owned)
    if classify(snapshot(), root, os.getpid()):
        raise RuntimeError('새 잔존 프로세스가 발견되어 시작을 중단합니다.')
    os.set_inheritable(fd, True)
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError) as exc:
        print(f'ROS 시작 중단: {exc}', file=sys.stderr)
        sys.exit(1)
