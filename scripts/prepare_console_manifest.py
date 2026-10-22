#!/usr/bin/env python3
"""Render deployment-local data paths for Console, without building or booting.

Package discovery uses manifest-relative paths in the public manifest. Runtime
data paths must be absolute because providers run in their own directories.
No .env or machine credentials are read. Hardware/model settings remain for
the operator to configure in Console.
"""
import argparse
import copy
from pathlib import Path

import yaml


def render(document, root):
    root = Path(root).resolve()

    def replace(value):
        if isinstance(value, str):
            return value.replace('${ROBONIX_DEPLOY_DIR}', str(root))
        if isinstance(value, list):
            return [replace(item) for item in value]
        if isinstance(value, dict):
            return {key: replace(item) for key, item in value.items()}
        return value

    result = replace(copy.deepcopy(document))
    # The output may live outside the checkout. Resolve only fields whose
    # contract is explicitly manifest-relative; leave arbitrary config alone.
    for section in ('primitive', 'service', 'skill'):
        for entry in result.get(section, []):
            path = entry.get('path')
            if path and '$' not in path and not Path(path).is_absolute():
                entry['path'] = str(root / path)
    soma = result.get('system', {}).get('soma', {})
    for field in ('robot_yaml', 'config'):
        path = soma.get(field)
        if isinstance(path, str) and '$' not in path and not Path(path).is_absolute():
            soma[field] = str(root / path)
    result.setdefault('env', {})['ROBONIX_DEPLOY_DIR'] = str(root)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        help='New file to create; defaults to robonix_manifest.console.yaml')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = root / 'robonix_manifest.yaml'
    output = args.output or root / 'robonix_manifest.console.yaml'
    result = render(yaml.safe_load(source.read_text()), root)
    # Exclusive creation protects an existing operator-edited deployment.
    with output.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    print(f'Created {output.resolve()}')
    print('Configure machine variables in Console; no dependencies installed or robot started.')


if __name__ == '__main__':
    main()
