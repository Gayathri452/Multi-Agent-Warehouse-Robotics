#!/usr/bin/env python3
"""Set Gazebo Harmonic's saved scene camera to a top-down warehouse view."""

import argparse
import os
from pathlib import Path
import re
import shutil
import tempfile


DEFAULT_CONFIG = Path.home() / '.gz' / 'sim' / '8' / 'gui.config'
TOP_CAMERA_POSE = '0 0 22 0 1.57079632679 0'
SCENE_PLUGIN = re.compile(
    r'<plugin\b(?=[^>]*(?:\bname\s*=\s*["\']3D View["\']|'
    r'\bfilename\s*=\s*["\'](?:GzScene3D|MinimalScene)["\']))'
    r'[^>]*>.*?</plugin>',
    re.DOTALL,
)


def remove_legacy_wrapper(text):
    match = re.fullmatch(
        r'\s*(<\?xml[^?]*\?>\s*)?<gui(?:\s[^>]*)?>(.*)</gui>\s*',
        text,
        re.DOTALL,
    )
    if match is None:
        return text
    return (match.group(1) or '') + match.group(2).strip() + '\n'


def set_camera_pose(text):
    text = remove_legacy_wrapper(text)
    scene = SCENE_PLUGIN.search(text)
    if scene is None:
        raise ValueError('No Gazebo 3D scene plugin was found in the GUI config')

    plugin = scene.group(0)
    camera_pose = re.compile(r'<camera_pose\b[^>]*>.*?</camera_pose>', re.DOTALL)
    replacement = '<camera_pose>%s</camera_pose>' % TOP_CAMERA_POSE
    if camera_pose.search(plugin):
        plugin = camera_pose.sub(replacement, plugin, count=1)
    else:
        closing = re.search(r'(?m)^([ \t]*)</plugin>[ \t]*$', plugin)
        if closing is None:
            raise ValueError('Could not locate the end of the Gazebo 3D scene plugin')
        indent = closing.group(1)
        insertion = '%s  %s\n%s</plugin>' % (indent, replacement, indent)
        plugin = plugin[:closing.start()] + insertion + plugin[closing.end():]

    return text[:scene.start()] + plugin + text[scene.end():]


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

    original = config_path.read_text(encoding='utf-8')
    try:
        updated = set_camera_pose(original)
    except ValueError as error:
        parser.error(f'{error} in {config_path}')

    backup_path = config_path.with_name(config_path.name + '.bak')
    if not backup_path.exists():
        shutil.copy2(config_path, backup_path)

    fd, temp_name = tempfile.mkstemp(
        prefix=config_path.name + '.', dir=config_path.parent)
    try:
        with os.fdopen(fd, 'wb') as config_file:
            config_file.write(updated.encode('utf-8'))
        os.replace(temp_name, config_path)
    except Exception:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
        raise

    print(f'Saved top-down camera pose to {config_path}')
    print(f'Original GUI config backup: {backup_path}')
    print('Close and restart Gazebo for the new view to take effect.')


if __name__ == '__main__':
    main()
