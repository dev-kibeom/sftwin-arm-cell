"""Expand an Assembly for ROS resource_retriever without changing Isaac inputs."""
from pathlib import Path

import xacro


def load_robot_description(source, mesh_root):
    document = xacro.process_file(str(source), mappings={'mesh_root': str(mesh_root)})
    # geometric_shapes/resource_retriever requires a URI, unlike Isaac's importer.
    for mesh in document.getElementsByTagName('mesh'):
        filename = mesh.getAttribute('filename')
        if Path(filename).is_absolute():
            mesh.setAttribute('filename', Path(filename).resolve().as_uri())
    return document.toxml()
