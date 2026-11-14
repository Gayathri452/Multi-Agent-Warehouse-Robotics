#!/usr/bin/env python3
"""Set Gazebo Harmonic's saved scene camera to a top-down warehouse view."""

import argparse
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET


DEFAULT_CONFIG = Path.home() / '.gz' / 'sim' / '8' / 'gui.config'
TOP_CAMERA_POSE = '0 0 22 0 1.57079632679 0'


def load_config(path):
    try:
        return ET.parse(path)
    except ET.ParseError:
        text = path.read_text(encoding='utf-8')
        text = re.sub(r'<\?xml\s+[^?]*\?>', '', text)
        root = ET.fromstring(f'<gui>{text}</gui>')
        nested_gui = root.find('gui')
        if nested_gui is not None:
            root.attrib.update(nested_gui.attrib)
            children = list(root)
            root.remove(nested_gui)
            for index, child in enumerate(list(nested_gui)):
                root.insert(index, child)
            for child in children:
                if child is not nested_gui:
                    root.append(child)
        return ET.ElementTree(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path, default=DEFAULT_CONFIG,
        help='Gazebo GUI config file (default: %(default)s)')
    config_path = parser.parse_args().config
    if not config_path.is_file():
        parser.error(
            f'{config_path} not found. Start Gazebo once to create its default '
            'GUI config, then rerun this script.')

    try:
        tree = load_config(config_path)
    except ET.ParseError as error:
        parser.error(f'Cannot parse Gazebo GUI config {config_path}: {error}')
    scene = next(
        (plugin for plugin in tree.getroot().iter('plugin')
         if plugin.get('name') == '3D View'
         or plugin.get('filename') in ('GzScene3D', 'MinimalScene')),
        None,
    )
    if scene is None:
        parser.error(f'No Gazebo 3D scene plugin found in {config_path}')

    camera_pose = scene.find('camera_pose')
    if camera_pose is None:
        camera_pose = ET.SubElement(scene, 'camera_pose')
    camera_pose.text = TOP_CAMERA_POSE
    ET.indent(tree, space='  ')

    fd, temp_name = tempfile.mkstemp(
        prefix=config_path.name + '.', dir=config_path.parent)
    try:
        with os.fdopen(fd, 'wb') as config_file:
            tree.write(config_file, encoding='utf-8', xml_declaration=True)
        os.replace(temp_name, config_path)
    except Exception:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
        raise

    print(f'Saved top-down camera pose to {config_path}')
    print('Close and restart Gazebo for the new view to take effect.')


if __name__ == '__main__':
    main()
