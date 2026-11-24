from types import SimpleNamespace
import unittest
from unittest.mock import patch
import fire_serial_bridge

from fire_serial_bridge import mock_target_message, start_mock_session, depth_message
from realsense_backend import DepthScene, DepthDetection


class FakeSerial:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.sent = []

    def write(self, data):
        self.sent.append(data)

    def readline(self):
        return next(self.lines, b'')


class MockProtocolTests(unittest.TestCase):
    def test_depth_mock_requires_wait_capability_before_start(self):
        link = FakeSerial([b'CAPS,MOCK_BASE_V1\n'])
        with self.assertRaisesRegex(RuntimeError, 'RANGE_WAIT_V1'):
            start_mock_session(link, timeout=0.001, require_range_wait=True)
        self.assertNotIn(b'MOCK_START\n', link.sent)

    def test_wait_capability_can_be_on_another_caps_line(self):
        link = FakeSerial([b'CAPS,FIRE,RANGE_WAIT_V1\n', b'CAPS,MOCK_BASE_V1\n',
                           b'MOCK_READY,80MM_WHEELS\n'])
        start_mock_session(link, timeout=0.1, require_range_wait=True)
        self.assertEqual(link.sent, [b'CAPS\n', b'MOCK_START\n'])

    def test_unknown_distance_is_distinct_from_clear_fire_in_both_protocols(self):
        unknown = DepthScene((DepthDetection((0, 0, 100, 100), 0.9, (10, 10, 20, 20),
                                             None, None, 'Insufficient valid depth'),))
        self.assertEqual(depth_message(unknown), 'RANGE_UNKNOWN\n')
        self.assertEqual(depth_message(unknown, 17), 'RANGE_UNKNOWN\n')
        self.assertEqual(depth_message(DepthScene(())), 'FIRE,0,0,0\n')
        self.assertEqual(depth_message(DepthScene(()), 18), 'TARGET,18,0,0,0,0\n')

    def test_caps_is_retried_if_initial_query_is_lost_during_boot(self):
        class BootingSerial:
            now = 0.0
            def __init__(self):
                self.sent = []
                self.pending = []
                self.queries = 0
            def write(self, data):
                self.sent.append(data)
                if data == b'CAPS\n':
                    self.queries += 1
                    if self.now >= 5.0:
                        self.pending.append(b'CAPS,MOCK_BASE_V1,TARGET,MOCK_START\n')
                elif data == b'MOCK_START\n':
                    self.pending.append(b'MOCK_READY,80MM_WHEELS\n')
            def readline(self):
                self.now += 0.1
                return self.pending.pop(0) if self.pending else b'BOOT\n'
        link = BootingSerial()
        with patch.object(fire_serial_bridge.time, 'monotonic', side_effect=lambda: link.now):
            start_mock_session(link)
        self.assertGreater(link.queries, 1)
        self.assertEqual(link.sent.count(b'MOCK_START\n'), 1)

    def test_timeout_reports_actual_serial_response(self):
        link = FakeSerial([b'OTHER_FIRMWARE\n'])
        with self.assertRaisesRegex(RuntimeError, 'Received: OTHER_FIRMWARE'):
            start_mock_session(link, timeout=0.001)
        self.assertNotIn(b'MOCK_START\n', link.sent)

    def test_capability_check_then_explicit_mock_start(self):
        link = FakeSerial([b'BOOT\n', b'CAPS,FIRE,STOP\n',
                           b'CAPS,MOCK_BASE_V1,TARGET,MOCK_START\n',
                           b'MOCK_READY,80MM_WHEELS\n'])
        start_mock_session(link, timeout=0.1)
        self.assertEqual(link.sent, [b'CAPS\n', b'MOCK_START\n'])

    def test_physical_firmware_is_not_started(self):
        link = FakeSerial([b'CAPS,FIRE,STOP,ELEVATION_ONLY\n'])
        with self.assertRaisesRegex(RuntimeError, 'does not advertise'):
            start_mock_session(link, timeout=0.001)
        self.assertEqual(link.sent, [b'CAPS\n'])

    def test_faulted_firmware_cannot_be_restarted(self):
        link = FakeSerial([b'CAPS,MOCK_BASE_V1\n', b'ERR,MOCK_START,NOT_ALLOWED\n'])
        with self.assertRaisesRegex(RuntimeError, 'reset ESP32'):
            start_mock_session(link, timeout=0.1)

    def test_atomic_target_has_matching_distance_and_raw_bearing(self):
        target = SimpleNamespace(distance_m=0.8, confidence=0.95, yaw_error_deg=-20)
        self.assertEqual(mock_target_message(17, target),
                         'TARGET,17,1,0.8000,0.9500,-20.000\n')
        self.assertEqual(mock_target_message(18, None), 'TARGET,18,0,0,0,0\n')


if __name__ == '__main__':
    unittest.main()
