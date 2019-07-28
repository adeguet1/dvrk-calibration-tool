import unittest
import numpy as np

class TestCalibration(unittest.TestCase):

    def test_distance(self):
        A, B, C = (0.5, 0.6, 0.7)
        pt = np.array([0.5, 0.6, 0.7])

        direction = np.array([A, B, -1])
        normal = direction / np.linalg.norm(direction)
        dist = np.dot(normal, pt - np.array([0, 0, C]))
        projection = pt - dist * normal
        
        self.assertAlmostEqual(
            A * projection[0] +
            B * projection[1] +
            C - projection[2], 0
        )
