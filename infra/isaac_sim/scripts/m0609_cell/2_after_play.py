"""Bootstrap gripper physics runtime and Simulation UI Hub after Play."""

import importlib
import shared.role_entrypoint as role_entrypoint

importlib.reload(role_entrypoint).run("after_play", globals())
