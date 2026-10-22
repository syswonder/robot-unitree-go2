"""Deployment relocation regression; no ROS, subprocess builds, or hardware."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'prepare_console_manifest', ROOT / 'scripts/prepare_console_manifest.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConsolePortabilityTests(unittest.TestCase):
    def setUp(self):
        self.document = yaml.safe_load((ROOT / 'robonix_manifest.yaml').read_text())

    def test_local_packages_resolve_without_deployment_environment(self):
        local = []
        for section in ('primitive', 'service', 'skill'):
            for entry in self.document.get(section, []):
                path = entry.get('path', '')
                if not path or path.startswith('${ROBONIX_SOURCE_PATH}'):
                    continue
                self.assertNotIn('$', path, entry['name'])
                self.assertFalse(Path(path).is_absolute(), entry['name'])
                local.append((entry['name'], ROOT / path / 'package_manifest.yaml'))
        self.assertEqual(len(local), 7)
        for name, manifest in local:
            self.assertTrue(manifest.is_file(), f'{name}: initialize pinned submodules')

    def test_soma_sidecar_resolves_without_environment(self):
        path = self.document['system']['soma']['robot_yaml']
        self.assertEqual(path, './soma.yaml')
        self.assertTrue((ROOT / path).is_file())

    def test_render_relocates_data_and_packages_but_preserves_user_variables(self):
        before = yaml.safe_dump(self.document)
        with tempfile.TemporaryDirectory(prefix='go2 relocated ') as folder:
            root = Path(folder)
            result = module.render(self.document, root)
            self.assertEqual(result['env']['SCENE_DATA_DIR'], str(root / 'rbnx-build/data/scene'))
            self.assertEqual(result['env']['ROBONIX_DEPLOY_DIR'], str(root))
            self.assertEqual(result['system']['soma']['robot_yaml'], str(root / 'soma.yaml'))
            chassis = result['primitive'][0]
            self.assertEqual(chassis['path'], str(root / 'packages/go2_chassis'))
            self.assertEqual(chassis['config'], self.document['primitive'][0]['config'])
            self.assertNotIn('${ROBONIX_DEPLOY_DIR}', yaml.safe_dump(result))
            self.assertIn('${GO2_NETWORK_INTERFACE}', yaml.safe_dump(result))
            self.assertIn('${ROBONIX_SOURCE_PATH}/services/speech', yaml.safe_dump(result))
        self.assertEqual(yaml.safe_dump(self.document), before)


if __name__ == '__main__':
    unittest.main()
