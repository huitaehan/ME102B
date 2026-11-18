from types import SimpleNamespace
import unittest

from fire_serial_bridge import mock_target_message, start_mock_session


class FakeSerial:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.sent = []

    def write(self, data):
        self.sent.append(data)

    def readline(self):
        return next(self.lines, b'')


class MockProtocolTests(unittest.TestCase):
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
