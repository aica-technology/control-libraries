import numpy as np
import state_representation as sr
import unittest
from datetime import timedelta
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from dynamical_systems.exceptions import EmptyAttractorError, EmptyBaseFrameError


class TestCartesianPointAttractor(unittest.TestCase):

    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_empty_constructor(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        attractor = sr.CartesianState.Identity("CAttractor", "A")

        self.assertTrue(ds.get_parameter_value("attractor").is_empty())
        self.assertTrue(ds.get_base_frame().is_empty())
        ds.set_parameter(sr.Parameter("attractor", attractor, sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE))
        self.assertFalse(ds.get_parameter_value("attractor").is_empty())
        self.assertFalse(ds.get_base_frame().is_empty())
        self.assertEqual(ds.get_base_frame().get_name(), attractor.get_reference_frame())
        self.assertEqual(ds.get_base_frame().get_reference_frame(), attractor.get_reference_frame())
        self.assert_np_array_equal(ds.get_base_frame().get_transformation_matrix(), np.eye(4))

    def test_is_compatible(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        state1 = sr.CartesianState.Identity("B", "A")
        state2 = sr.CartesianState("D", "C")
        state3 = sr.CartesianState("C", "A")
        state4 = sr.CartesianState("C", "B")

        with self.assertRaises(EmptyBaseFrameError):
            ds.evaluate(state1)

        ds.set_base_frame(state1)
        with self.assertRaises(sr.exceptions.IncompatibleReferenceFramesError):
            ds.evaluate(state2)
        with self.assertRaises(sr.exceptions.EmptyStateError):
            ds.evaluate(state3)
        with self.assertRaises(EmptyAttractorError):
            ds.evaluate(state4)

        ds.set_parameter(sr.Parameter("attractor", sr.CartesianState.Identity("CAttractor", "A"),
                                      sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE))
        self.assertTrue(ds.is_compatible(state1))
        self.assertFalse(ds.is_compatible(state2))
        self.assertTrue(ds.is_compatible(state3))
        self.assertTrue(ds.is_compatible(state4))

    def test_pose(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        target = sr.CartesianPose.Random("B")
        ds.set_parameter(sr.Parameter("attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE))

        current_pose = sr.CartesianPose.Identity("B")
        for i in range(100):
            twist = sr.CartesianTwist(ds.evaluate(current_pose))
            current_pose += timedelta(milliseconds=100) * twist

        self.assertTrue(current_pose.dist(target, sr.CartesianStateVariable.POSITION) < 1e-3)
        self.assertTrue(current_pose.dist(target, sr.CartesianStateVariable.ORIENTATION) < 1e-3)

    def test_stacked_moving_reference_frames(self):
        AinWorld = sr.CartesianState.Random("A")
        BinA = sr.CartesianState.Random("B", "A")
        CinA = sr.CartesianState(sr.CartesianPose.Random("C", "A"))

        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        ds.set_parameter(sr.Parameter("attractor", BinA, sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE))

        twist = sr.CartesianTwist(ds.evaluate(CinA))

        CinA.set_linear_velocity(np.random.rand(3, 1))
        CinA.set_angular_velocity(np.random.rand(3, 1))
        twist2 = sr.CartesianTwist(ds.evaluate(CinA))

        self.assertTrue(np.linalg.norm(twist.data()) - np.linalg.norm(twist2.data()) < 1e-5)

        twist = AinWorld * ds.evaluate(CinA)

        ds.set_base_frame(AinWorld)
        CinWorld = AinWorld * CinA
        twist2 = ds.evaluate(CinWorld)

        self.assertEqual(twist.get_reference_frame(), AinWorld.get_reference_frame())
        self.assertEqual(twist.get_reference_frame(), twist2.get_reference_frame())
        self.assertTrue(np.linalg.norm(twist.data()) - np.linalg.norm(twist2.data()) < 1e-5)


if __name__ == '__main__':
    unittest.main()
