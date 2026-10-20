"""Floor commands are forwarded to pi5-stm32-slam scripts/floors.py; no robot is contacted."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import remote_ops


class FloorCommandTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.project = Path(folder.name) / 'pi5 stm32'
        (self.project / 'scripts').mkdir(parents=True)
        (self.project / 'scripts/floors.py').touch()
        patcher = patch.object(remote_ops, 'ROOT', self.project)
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_with(self, stdout, returncode=0):
        result = subprocess.CompletedProcess([], returncode, stdout, '')
        return patch.object(remote_ops.subprocess, 'run', return_value=result)

    def test_switch_passes_level_and_domain(self):
        with self.run_with('{"level": 2, "map": "tang2"}\n') as run:
            report = remote_ops.floor_command('floor-switch', 7, level=2)
        self.assertEqual(report, {'level': 2, 'map': 'tang2'})
        command = run.call_args.args[0]
        self.assertEqual(command[4:], [str(self.project), 'switch', '--level', '2'])
        self.assertEqual(run.call_args.kwargs['env']['ROS_DOMAIN_ID'], '7')
        self.assertGreaterEqual(run.call_args.kwargs['timeout'], 120)

    def test_register_forwards_maps_and_label(self):
        with self.run_with('{"floors": []}') as run:
            remote_ops.floor_command('floor-register', 0, level=1, map_name='tang1', map3d='tang1', label='Tầng 1')
        self.assertEqual(run.call_args.args[0][5:],
                         ['register', '--level', '1', '--map', 'tang1', '--map3d', 'tang1', '--label', 'Tầng 1'])

    def test_script_error_is_raised(self):
        with self.run_with(json.dumps({'error': 'Floor 3 is not registered'}), returncode=1):
            with self.assertRaisesRegex(RuntimeError, 'Floor 3 is not registered'):
                remote_ops.floor_command('floor-switch', 0, level=3)

    def test_missing_script_explains_update(self):
        (self.project / 'scripts/floors.py').unlink()
        with self.assertRaisesRegex(RuntimeError, 'floors.py'):
            remote_ops.floor_command('floors', 0)


if __name__ == '__main__':
    unittest.main()
