import os
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_guard import Process, classify, stop_processes, snapshot


def test_legacy_tree_owned_but_other_checkout_blocks(tmp_path):
    root = tmp_path / 'cell'
    launch = Process(10, 1, 100, ('/opt/ros/humble/bin/ros2', 'launch',
                                 'arm_cell_bringup', 'arm_cell.launch.py'), root)
    child = Process(11, 10, 101, (str(root / 'ros2_ws/install/p/lib/p/orchestration_node'),), root)
    assert classify({10: launch, 11: child}, root, 99) == {10: launch, 11: child}
    foreign = Process(12, 1, 102, ('/other/ros2_ws/install/arm_cell_test/lib/p/task_execution_node',), Path('/other'))
    with pytest.raises(RuntimeError, match='12'):
        classify({10: launch, 11: child, 12: foreign}, root, 99)


def test_pid_reuse_is_not_signalled():
    process = Process(os.getpid(), 1, -1, (), Path('/'))
    stop_processes({process.pid: process}, timeouts=(0, 0, 0))


def test_real_process_is_stopped():
    process = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        stop_processes({process.pid: snapshot()[process.pid]}, timeouts=(1, 1, 1))
        assert process.wait(timeout=2) is not None
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_unrelated_process_is_preserved(tmp_path):
    unrelated = Process(20, 1, 200, ('python3', 'worker.py'), tmp_path)
    assert classify({20: unrelated}, tmp_path, 99) == {}


def test_signal_escalates_when_interrupt_is_ignored():
    process = subprocess.Popen([sys.executable, '-u', '-c',
        'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); '
        'print("ready", flush=True); time.sleep(60)'], stdout=subprocess.PIPE, text=True)
    try:
        assert process.stdout.readline().strip() == 'ready'
        stop_processes({process.pid: snapshot()[process.pid]}, timeouts=(0.1, 1, 1))
        assert process.wait(timeout=2) == -15
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdout.close()


def test_guard_exec_preserves_pid_and_blocks_concurrent_start(tmp_path):
    script = Path(__file__).resolve().parents[1] / 'ros_guard.py'
    command = [sys.executable, str(script), sys.executable, '-u', '-c',
               'import os,time; print(os.getpid(), flush=True); time.sleep(60)']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert process.stdout.readline().strip() == str(process.pid)
        second = subprocess.run(command, capture_output=True, text=True, timeout=5)
        assert second.returncode == 1
        assert '먼저 종료' in second.stderr
    finally:
        process.terminate()
        process.wait(timeout=3)
        process.stdout.close()
        process.stderr.close()


@pytest.mark.parametrize('argv', [
    ('/opt/ros/humble/lib/moveit_ros_move_group/move_group',),
    ('python3', '/other/ros2_ws/install/arm_cell_motion/lib/arm_cell_motion/load_static_scene.py'),
])
def test_orphaned_generic_and_python_ros_nodes_block(tmp_path, argv):
    orphan = Process(30, 1, 300, argv, tmp_path)
    with pytest.raises(RuntimeError, match='30'):
        classify({30: orphan}, tmp_path, 99)


def test_editor_of_installed_file_is_unrelated(tmp_path):
    editor = Process(31, 1, 301, ('vim', '/other/ros2_ws/install/arm_cell_motion/config.yaml'), tmp_path)
    assert classify({31: editor}, tmp_path, 99) == {}
