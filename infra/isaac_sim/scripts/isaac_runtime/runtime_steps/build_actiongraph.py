"""Invoke the selected Cell's existing action_graph_builder implementation."""
from isaac_runtime.runtime_workflow import call_selected
call_selected(runtime_configuration['cell']['isaac']['action_graph_builder'],
              configuration=runtime_configuration, namespace=globals())
