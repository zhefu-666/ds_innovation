import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import io

spec = importlib.util.spec_from_file_location('monitor', Path(__file__).with_name('monitor.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class MonitorTests(unittest.TestCase):
    def test_no_log(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(m.read_logs(Path(d))['lines'], [])

    def test_new_run_and_partial_line(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/'match-main-20261012-000000-1.log').write_text('old\n')
            (p/'match-main-20261012-010000-2.log').write_text('[CONTROL_CHANGE] 新一趟\nnormal\n未写完')
            v = m.read_logs(p)
            self.assertEqual(v['lines'], ['[CONTROL_CHANGE] 新一趟', 'normal'])
            self.assertEqual(v['events'], ['[CONTROL_CHANGE] 新一趟'])
            (p/'match-main-20261012-020000-3.log').write_text('next\n')
            self.assertEqual(m.read_logs(p)['lines'], ['next'])

    def test_bounded_tail_and_symlink(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            f=p/'match-main-20261012-010000-2.log';f.write_text('x'*300000+'\n[CONTROL_CHANGE] tail\n')
            (p/'match-main-20261012-020000-3.log').symlink_to(f)
            v=m.read_logs(p)
            self.assertEqual(v['lines'], ['[CONTROL_CHANGE] tail'])
            self.assertEqual(v['name'], f.name)

    def test_live_to_offline_drops_state(self):
        with tempfile.TemporaryDirectory() as d:
            source=m.Source(Path(d),'http://127.0.0.1:8080')
            payload={'health':{'source':'rescue_upper_host','camera_ok':True,'robot_data_connected':True},'state':{'name':'RUSH'}}
            with patch.object(m.OPENER, 'open', return_value=io.BytesIO(json.dumps(payload).encode())):
                source.poll()
            self.assertTrue(source.snapshot()['camera_live'])
            with patch.object(m.OPENER, 'open', side_effect=OSError('offline')):
                source.poll()
            self.assertFalse(source.snapshot()['live'])
            self.assertNotIn('telemetry',source.snapshot())

    def test_stale_snapshot_is_not_live(self):
        with tempfile.TemporaryDirectory() as d:
            source=m.Source(Path(d),'http://127.0.0.1:8080')
            payload={'health':{'source':'rescue_upper_host','camera_ok':True,'robot_data_connected':False}}
            with patch.object(m.OPENER, 'open', return_value=io.BytesIO(json.dumps(payload).encode())):
                source.poll()
            self.assertFalse(source.snapshot()['camera_live'])

    def test_unrelated_endpoint_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            source=m.Source(Path(d),'http://127.0.0.1:8080')
            with patch.object(m.OPENER,'open',return_value=io.BytesIO(b'{"health":{"source":"other"}}')):
                source.poll()
            self.assertFalse(source.snapshot()['live'])
            self.assertIn('unexpected',source.snapshot()['error'])

if __name__=='__main__':
    unittest.main()
