"""Run with: python -m unittest discover -s vision -p 'test_*.py'."""
import math
from types import SimpleNamespace
import unittest

from fire_serial_bridge import solve_request, solver


class SolverProtocolTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(
            jet_speed_m_s=5.0, target_height_m=0.2, nozzle_length_m=0.1,
            turret_x_m=0.05, turret_y_m=0.0, base_height_m=0.2)

    def test_angle_reaches_target_with_nozzle_offset(self):
        reply = solve_request('AIM_REQUEST,17,0.8,5,85', self.args)
        prefix, request_id, angle = reply.strip().split(',')
        self.assertEqual((prefix, request_id), ('AIM_RESULT', '17'))
        angle = float(angle)
        self.assertTrue(5 <= angle <= 85)
        self.assertAlmostEqual(
            solver.miss(angle, 0.8, 0.2, 5.0, 0.1, 0.05, 0.0, 0.2),
            0.0, delta=1e-4)

    def test_unreachable_target_returns_matching_error(self):
        self.assertEqual(solve_request('AIM_REQUEST,18,100,5,85', self.args),
                         'AIM_ERROR,18\n')

    def test_missing_calibration_returns_error(self):
        self.args.jet_speed_m_s = None
        self.assertEqual(solve_request('AIM_REQUEST,19,0.8,5,85', self.args),
                         'AIM_ERROR,19\n')

    def test_bad_requests_do_not_produce_angles(self):
        for distance in ('nan', 'inf', '-1', '0'):
            with self.subTest(distance=distance):
                self.assertEqual(
                    solve_request(f'AIM_REQUEST,20,{distance},5,85', self.args),
                    'AIM_ERROR,20\n')
        self.assertIsNone(solve_request('FLAME,600,1,0', self.args))


if __name__ == '__main__':
    unittest.main()
