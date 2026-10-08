#!/usr/bin/env python3
"""Read-only installed mesh availability gate; does not initialize ROS."""
import json
import os
from pathlib import Path
import subprocess
import urllib.parse
import xml.etree.ElementTree as ET


def package_root(package):
    return Path(subprocess.check_output(['rospack', 'find', package], text=True).strip()).resolve()


def readable(path, root):
    path = path.resolve()
    if root not in path.parents or not path.is_file() or path.stat().st_size == 0 or not os.access(path, os.R_OK):
        raise AssertionError('missing/unreadable installed resource: ' + str(path))
    return path


result = {}
for package, model in [('fs150_description', 'fs150'), ('scout_description', 'scout'), ('mecanum_description', 'mecanum')]:
    root = package_root(package)
    urdf = readable(root / 'urdf' / (model + '_visual.urdf'), root)
    meshes, textures = set(), set()
    for element in ET.parse(urdf).iter('mesh'):
        uri = urllib.parse.urlparse(element.attrib['filename'])
        if uri.scheme != 'package' or not uri.netloc:
            raise AssertionError('canonical visual mesh must be package-relative')
        owner = package_root(uri.netloc)
        mesh = readable(owner / urllib.parse.unquote(uri.path.lstrip('/')), owner)
        meshes.add(str(mesh))
        if mesh.suffix.lower() == '.dae':
            for image in ET.parse(mesh).findall('.//{*}library_images/{*}image/{*}init_from'):
                value = urllib.parse.unquote((image.text or '').strip())
                texture = readable(mesh.parent / value, owner)
                textures.add(str(texture))
    if not meshes:
        raise AssertionError('canonical visual URDF has no mesh resources: ' + package)
    result[package] = dict(mesh_files=len(meshes), texture_files=len(textures))
print(json.dumps(dict(ok=True, scope='installed resource availability; not rendering acceptance', resources=result), sort_keys=True))
