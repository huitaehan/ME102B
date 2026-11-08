import unittest

from imx500_backend import observation_from_outputs


class Imx500Tests(unittest.TestCase):
    def decode(self, outputs, units='pixels'):
        return observation_from_outputs(
            outputs, box_order='yxyx', box_units=units, input_height=640,
            flame_height_cm=5, focal_length_px=600)

    def test_missing_inference_is_not_clear(self):
        self.assertIsNone(self.decode(None))

    def test_count_excludes_padded_detections(self):
        self.assertEqual(self.decode([[[0, 0, 100, 100]], [0.9], [0], [0]]),
                         'FIRE,0,0,0\n')

    def test_normalized_and_pixel_distance_match(self):
        pixels = self.decode([[[0, 0, 60, 30]], [0.9], [0], [1]])
        normalized = self.decode(
            [[[0, 0, 60 / 640, 30 / 640]], [0.9], [0], [1]], 'normalized')
        self.assertEqual(pixels, 'FIRE,1,0.5000,0.9000\n')
        self.assertEqual(pixels, normalized)

    def test_invalid_count_is_rejected(self):
        with self.assertRaises(ValueError):
            self.decode([[[0, 0, 60, 30]], [0.9], [0], [2]])


if __name__ == '__main__':
    unittest.main()
