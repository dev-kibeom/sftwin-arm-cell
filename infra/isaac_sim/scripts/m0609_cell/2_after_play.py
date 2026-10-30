"""Bootstrap gripper physics runtime and Simulation UI Hub after Play."""

import importlib
from shared import role_entrypoint

importlib.reload(role_entrypoint).run("after_play", globals())
