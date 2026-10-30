"""Bootstrap physics runtime after Play without loading the UI Hub."""

import importlib
from shared import role_entrypoint

importlib.reload(role_entrypoint).run("after_play_headless", globals())
