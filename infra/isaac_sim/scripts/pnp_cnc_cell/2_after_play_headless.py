"""Run common Isaac after_play_headless setup in the retained Script Editor namespace."""
import importlib
from isaac_runtime import role_entrypoint
importlib.reload(role_entrypoint).run("after_play_headless", globals())
