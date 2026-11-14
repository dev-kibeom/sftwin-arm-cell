"""Invoke the selected Cell's existing camera_snapshot implementation."""
from isaac_runtime.runtime_workflow import call_selected
call_selected(runtime_configuration['cell']['isaac']['camera_snapshot'],
              configuration=runtime_configuration, namespace=globals())
