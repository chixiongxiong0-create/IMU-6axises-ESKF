import math
import unittest

from tools.attitude_viewer.geometry import rotation_matrix


class RotationMatrixTests(unittest.TestCase):
    def test_identity_preserves_frd_axes(self):
        self.assertEqual(
            rotation_matrix((1.0, 0.0, 0.0, 0.0)),
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        )

    def test_positive_ninety_degree_yaw_maps_forward_north_to_east(self):
        matrix = rotation_matrix((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
        forward = tuple(row[0] for row in matrix)
        for actual, expected in zip(forward, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected)

    def test_positive_ninety_degree_roll_maps_right_to_down(self):
        matrix = rotation_matrix((math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0))
        right = tuple(row[1] for row in matrix)
        for actual, expected in zip(right, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)

    def test_positive_ninety_degree_pitch_maps_forward_to_up(self):
        matrix = rotation_matrix((math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0))
        forward = tuple(row[0] for row in matrix)
        for actual, expected in zip(forward, (0.0, 0.0, -1.0)):
            self.assertAlmostEqual(actual, expected)

    def test_rotation_axes_stay_orthonormal(self):
        quaternion = (0.5, 0.5, 0.5, 0.5)
        matrix = rotation_matrix(quaternion)
        columns = [tuple(row[index] for row in matrix) for index in range(3)]
        for left in range(3):
            for right in range(3):
                dot = sum(a * b for a, b in zip(columns[left], columns[right]))
                self.assertAlmostEqual(dot, float(left == right))


if __name__ == "__main__":
    unittest.main()
