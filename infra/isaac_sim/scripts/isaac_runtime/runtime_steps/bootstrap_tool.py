"""Initialize the selected Tool after Physics has started."""
from isaac_runtime.runtime_workflow import call_selected
call_selected(runtime_configuration['tool']['runtime']['isaac_bootstrap'],
              configuration=runtime_configuration, namespace=globals())
