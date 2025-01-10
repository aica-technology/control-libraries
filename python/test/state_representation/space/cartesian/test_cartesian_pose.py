import unittest
import copy
from datetime import timedelta

from state_representation import CartesianPose, CartesianTwist
import numpy as np

class TestCartesianPose(unittest.TestCase):
    def assert_np_array_equal(self, a: np.array, b: np.array, places=5):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_constructors(self):
        CartesianPose("A", "B")
        A = CartesianPose("A", np.array([1, 2, 3]), "B")
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        self.assert_np_array_equal(A.get_orientation().elements, [1, 0, 0, 0])
        self.assert_np_array_equal(A.get_orientation_coefficients(), [1, 0, 0, 0])

        A = CartesianPose("A", np.array([1, 2, 3, 4]), "B")
        self.assert_np_array_equal(A.get_position(), [0, 0, 0])
        self.assert_np_array_equal(A.get_orientation_coefficients(), [1, 2, 3, 4] / np.linalg.norm([1, 2, 3, 4]))

        A = CartesianPose("A", np.array([1, 2, 3]), np.array([1, 2, 3, 4]), "B")
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        self.assert_np_array_equal(A.get_orientation().elements, [1, 2, 3, 4] / np.linalg.norm([1, 2, 3, 4]))

        B = CartesianPose().from_transformation_matrix(A.get_name(), A.get_transformation_matrix())
        self.assert_np_array_equal(B.get_pose(), A.get_pose())

    def test_copy(self):
        state = CartesianPose().Random("test")
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_name(), state_copy.get_name())
            self.assertEqual(state.get_reference_frame(), state_copy.get_reference_frame())
            self.assert_np_array_equal(state.data(), state_copy.data())

    def test_inverse(self):
        pose = CartesianPose.Random("A", "B")
        inv_pose = pose.inverse()
        self.assertIsInstance(inv_pose, CartesianPose)

    def test_differentiation(self):
        pose = CartesianPose.Random("test")
        pose.set_orientation([0, 1, 0, 0])
        dt1 = 0.1
        dt2 = timedelta(milliseconds=100)

        res1 = pose / dt2
        self.assertIsInstance(res1, CartesianTwist)
        self.assert_np_array_equal(pose.get_position(), dt1 * res1.get_linear_velocity())
        self.assert_np_array_equal(np.array([np.pi, 0, 0]), dt1 * res1.get_angular_velocity())

        res2 = pose.differentiate(dt1)
        self.assertIsInstance(res2, CartesianTwist)
        self.assert_np_array_equal(pose.get_position(), dt1 * res2.get_linear_velocity())
        self.assert_np_array_equal(np.array([np.pi, 0, 0]), dt1 * res2.get_angular_velocity())

if __name__ == '__main__':
    unittest.main()