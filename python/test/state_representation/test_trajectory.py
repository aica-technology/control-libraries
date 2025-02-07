import unittest
import datetime
from random import shuffle

from state_representation import (
    CartesianState,
    JointState,
    CartesianTrajectory,
    JointTrajectory,
    StateType
)

from state_representation.exceptions import (
    EmptyStateError,
    IncompatibleStatesError,
    IncompatibleReferenceFramesError,
    IncompatibleSizeError,
)


class TestState(unittest.TestCase):
    __num_random_tests = 10

    __cartesian_state_attributes = [
        "get_acceleration",
        "get_angular_acceleration",
        "get_angular_velocity",
        "get_force",
        "get_linear_acceleration",
        "get_linear_velocity",
        "get_orientation",
        "get_orientation_coefficients",
        "get_pose",
        "get_position",
        "get_torque",
        "get_twist",
        "get_wrench",
    ]

    __joint_state_attributes = [
        "get_accelerations",
        "get_positions",
        "get_torques",
        "get_velocities",
    ]

    def test_constructors(self):
        # empty trajectory constructors
        empty1 = CartesianTrajectory()
        self.assertEqual(empty1.get_type(), StateType.CARTESIAN_TRAJECTORY)
        self.assertEqual(empty1.get_name(), "")
        self.assertTrue(empty1.is_empty())
        self.assertEqual(empty1.get_reference_frame(), "")

        empty2 = CartesianTrajectory("test")
        self.assertEqual(empty2.get_type(), StateType.CARTESIAN_TRAJECTORY)
        self.assertEqual(empty2.get_name(), "test")
        self.assertTrue(empty2.is_empty())
        self.assertEqual(empty2.get_reference_frame(), "world")

        empty3 = CartesianTrajectory("test", "reference")
        self.assertEqual(empty3.get_type(), StateType.CARTESIAN_TRAJECTORY)
        self.assertEqual(empty3.get_name(), "test")
        self.assertTrue(empty3.is_empty())
        self.assertEqual(empty3.get_reference_frame(), "reference")

        empty4 = JointTrajectory()
        self.assertEqual(empty4.get_type(), StateType.JOINT_TRAJECTORY)
        self.assertEqual(empty4.get_name(), "")
        self.assertTrue(empty4.is_empty())

        empty5 = JointTrajectory("test")
        self.assertEqual(empty5.get_type(), StateType.JOINT_TRAJECTORY)
        self.assertEqual(empty5.get_name(), "test")
        self.assertTrue(empty5.is_empty())

        with self.assertRaises(EmptyStateError):
            CartesianTrajectory("test", CartesianState(), datetime.timedelta(seconds=0))

        # trajectory constructors with data
        dummy_cartesian_state = CartesianState.Random("world")
        data1 = CartesianTrajectory("test", dummy_cartesian_state, datetime.timedelta(seconds=1))
        self.assertEqual(data1.get_type(), StateType.CARTESIAN_TRAJECTORY)
        self.assertEqual(data1.get_name(), "test")
        self.assertFalse(data1.is_empty())
        self.assertEqual(data1.get_reference_frame(), "world")

        dummy_cartesian_state = CartesianState.Random("world")
        data2 = CartesianTrajectory("test", [dummy_cartesian_state] * 5, [datetime.timedelta(seconds=1)] * 5)
        self.assertEqual(data2.get_type(), StateType.CARTESIAN_TRAJECTORY)
        self.assertEqual(data2.get_name(), "test")
        self.assertFalse(data2.is_empty())
        self.assertEqual(data2.get_reference_frame(), "world")
        self.assertEqual(data2.get_size(), 5)

        dummy_joint_state = JointState.Random("robot", 7)
        data2 = JointTrajectory("test", dummy_joint_state, datetime.timedelta(seconds=1))
        self.assertEqual(data2.get_type(), StateType.JOINT_TRAJECTORY)
        self.assertEqual(data2.get_name(), "test")
        self.assertFalse(data2.is_empty())

        dummy_joint_state = JointState.Random("robot", 7)
        data2 = JointTrajectory("test", [dummy_joint_state] * 5, [datetime.timedelta(seconds=1)] * 5)
        self.assertEqual(data2.get_type(), StateType.JOINT_TRAJECTORY)
        self.assertEqual(data2.get_name(), "test")
        self.assertFalse(data2.is_empty())
        self.assertEqual(data2.get_size(), 5)

        with self.assertRaises(EmptyStateError):
            empty6 = JointTrajectory("test", JointState(), datetime.timedelta(seconds=0))

    def test_addremove_points(self):
        for TrajectoryT in [CartesianTrajectory, JointTrajectory]:
            trajectory = TrajectoryT("foo")

            points = []
            durations = []
            for i in range(self.__num_random_tests):
                points.append(
                    CartesianState.Random("foo")
                    if isinstance(trajectory, CartesianTrajectory)
                    else JointState.Random("foo", 25)
                )
                durations.append(datetime.timedelta(seconds=(i + 1) * 10))

            if isinstance(trajectory, JointTrajectory):
                trajectory.set_joint_names(points[0].get_names())

            for i, (point, duration) in enumerate(zip(points, durations)):
                trajectory.add_point(point, duration)
                self.assertEqual(trajectory.get_size(), i + 1)
                self.assert_state_equality(trajectory[i][0], points[i])
                self.assertEqual(trajectory[i][1], durations[i])

            for i in range(self.__num_random_tests):
                trajectory.delete_point(0)
                self.assertEqual(trajectory.get_size(), self.__num_random_tests - i - 1) 

            trajectory.add_points(points, durations)
            self.assertEqual(trajectory.get_size(), self.__num_random_tests)
            for i, (point, duration) in enumerate(zip(points, durations)):
                self.assert_state_equality(trajectory[i][0], points[i])
                self.assertEqual(trajectory[i][1], durations[i])

            shuffled_points = points.copy()
            shuffled_durations = durations.copy()
            shuffle(shuffled_points)
            shuffle(shuffled_durations)
            for i, (point, duration) in enumerate(zip(shuffled_points, shuffled_durations)):
                trajectory.set_point(point, duration, i)
                self.assert_state_equality(trajectory[i][0], shuffled_points[i])
                self.assertEqual(trajectory[i][1], shuffled_durations[i])

            shuffle(shuffled_points)
            shuffle(shuffled_durations)
            trajectory.set_points(shuffled_points, shuffled_durations)
            self.assertEqual(trajectory.get_size(), self.__num_random_tests)
            for i, (point, duration) in enumerate(zip(shuffled_points, shuffled_durations)):
                self.assert_state_equality(trajectory[i][0], shuffled_points[i])
                self.assertEqual(trajectory[i][1], shuffled_durations[i])

            insertion_point = (
                CartesianState.Random("foo")
                if isinstance(trajectory, CartesianTrajectory)
                else JointState.Random("foo", 25)
            )
            insert_idx = self.__num_random_tests // 2
            trajectory.insert_point(insertion_point, durations[0], insert_idx)
            self.assertEqual(trajectory.get_size(), self.__num_random_tests + 1)
            self.assert_state_equality(trajectory[insert_idx][0], insertion_point)
            self.assertEqual(trajectory[insert_idx][1], durations[0])

    def test_exceptions(self):
        for TrajectoryT in [CartesianTrajectory, JointTrajectory]:
            trajectory = TrajectoryT("foo")

            points = []
            durations = []
            for i in range(self.__num_random_tests):
                points.append(
                    CartesianState.Random("foo")
                    if isinstance(trajectory, CartesianTrajectory)
                    else JointState.Random("foo", 25)
                )
                durations.append(datetime.timedelta(seconds=(i + 1) * 10))

            if isinstance(trajectory, JointTrajectory):
                trajectory.set_joint_names(points[0].get_names())

            trajectory.add_points(points, durations)
            with self.assertRaises(IndexError):
                trajectory.get_point(self.__num_random_tests + 1)

            trajectory.add_point(points[0], durations[0])
            with self.assertRaises(IncompatibleSizeError):
                trajectory.set_points(points, durations)

            if isinstance(trajectory, CartesianTrajectory):
                with self.assertRaises(IncompatibleReferenceFramesError):
                    trajectory.add_point(CartesianState.Random("bar", "some_world"), durations[0])
                with self.assertRaises(IncompatibleReferenceFramesError):
                    trajectory.set_point(CartesianState.Random("bar", "some_world"), durations[0], 0)
                with self.assertRaises(EmptyStateError):
                    trajectory.add_point(CartesianState(), durations[0])
            else:
                with self.assertRaises(IncompatibleStatesError):
                    trajectory.add_point(JointState.Random("foo", 24), durations[0])
                with self.assertRaises(IncompatibleStatesError):
                    trajectory.set_point(JointState.Random("foo", 24), durations[0], 0)
                with self.assertRaises(EmptyStateError):
                    trajectory.add_point(JointState(), durations[0])

            with self.assertRaises(IndexError):
                trajectory[self.__num_random_tests + 1]

    def test_getters(self):
        for TrajectoryT in [CartesianTrajectory, JointTrajectory]:
            trajectory = TrajectoryT("foo")

            points = []
            durations = []
            for i in range(self.__num_random_tests):
                points.append(
                    CartesianState.Random("foo")
                    if isinstance(trajectory, CartesianTrajectory)
                    else JointState.Random("foo", 25)
                )
                durations.append(datetime.timedelta(seconds=(i + 1) * 10))

            if isinstance(trajectory, JointTrajectory):
                trajectory.set_joint_names(points[0].get_names())

            trajectory.add_points(points, durations)

            for i, (point, duration) in enumerate(zip(points, durations)):
                self.assert_state_equality(trajectory.get_point(i), point)
                self.assertEqual(trajectory.get_duration(i), duration)
                self.assert_state_equality(trajectory[i][0], point)
                self.assertEqual(trajectory[i][1], duration)

            times_from_start = trajectory.get_times_from_start()
            time_from_start = 0
            for i in range(len(times_from_start)):
                time_from_start += durations[i].total_seconds()
                self.assertAlmostEqual(time_from_start, times_from_start[i].total_seconds())
                self.assertAlmostEqual(time_from_start, times_from_start[i].total_seconds())
                self.assertAlmostEqual(trajectory.get_duration(i), durations[i])
            self.assertAlmostEqual(trajectory.get_trajectory_duration().total_seconds(), time_from_start)

            trajectory.reset()
            self.assertTrue(trajectory.is_empty())
            self.assertEqual(trajectory.get_size(), 0)
            with self.assertRaises(EmptyStateError):
                trajectory.get_durations()
            if isinstance(trajectory, CartesianTrajectory):
                self.assertEqual(trajectory.get_reference_frame(), "world")
            else:
                self.assertFalse(len(trajectory.get_joint_names()) == 0)

    def assert_state_equality(self, state1, state2):
        self.assertEqual(state1.get_type(), state2.get_type())
        self.assertEqual(state1.get_name(), state2.get_name())
        if isinstance(state1, CartesianState):
            for attr in self.__cartesian_state_attributes:
                for value1, value2 in zip(getattr(state1, attr)(), getattr(state2, attr)()):
                    self.assertAlmostEqual(value1, value2)
            self.assertEqual(state1.get_reference_frame(), state2.get_reference_frame())
        else:
            for attr in self.__joint_state_attributes:
                for value1, value2 in zip(getattr(state1, attr)(), getattr(state2, attr)()):
                    self.assertAlmostEqual(value1, value2)
            self.assertEqual(state1.get_names(), state2.get_names())


if __name__ == "__main__":
    unittest.main()
