"""Test-only in-process bridge to the native Task BT, with live fixture callbacks."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import uuid


def run_task_grasp_cycle(library_path, strategy, solution, callback):
    library_path = Path(library_path).resolve(strict=True)
    library = ctypes.CDLL(str(library_path))
    callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.c_char_p)
    errors = []

    def invoke(event, name):
        try:
            return int(callback(event, name.decode('utf-8')))
        except BaseException as error:
            # ctypes cannot propagate a Python exception through native frames.
            errors.append(error)
            return 0 if event == 3 else 9  # UNKNOWN observation / BACKEND_ERROR

    native_callback = callback_type(invoke)
    run = library.arm_cell_run_grasp_cycle
    run.argtypes = [ctypes.c_char_p, callback_type]
    run.restype = ctypes.c_int
    # The BT XML is runtime input separate from the native binary. Match the
    # first ament package prefix, just as the native factory does.
    config = None
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        prefix = Path(prefix)
        if (prefix / 'share/ament_index/resource_index/packages/arm_cell_task_execution').is_file():
            config = prefix / 'share/arm_cell_task_execution/config'
            break
    if config is None:
        raise RuntimeError('Task Execution overlay must be sourced for the fixture bridge')
    bt_hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                 for path in sorted(config.glob('*.xml'))}
    request = dict(strategy=strategy, solution=solution, execution_id=uuid.uuid4().hex)
    result = run(json.dumps(request).encode('utf-8'), native_callback)
    if errors:
        raise errors[0]
    if result != 0:
        raise RuntimeError(f'Common Task grasp cycle failed: result={result}')
    return dict(library=str(library_path), library_sha256=hashlib.sha256(library_path.read_bytes()).hexdigest(),
                execution_id=request['execution_id'], result_code=result, bt_sha256=bt_hashes)
