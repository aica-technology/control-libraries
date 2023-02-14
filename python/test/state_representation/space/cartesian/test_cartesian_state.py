import unittest
import copy

import numpy as np
from pyquaternion.quaternion import Quaternion
from numpy.testing import assert_array_equal, assert_array_almost_equal
from state_representation import State, CartesianState, StateType, CartesianStateVariable, CartesianPose, \
    CartesianTwist, CartesianAcceleration, CartesianWrench
from state_representation.exceptions import EmptyStateError, IncompatibleReferenceFramesError, IncompatibleSizeError, \
    NotImplementedError
from datetime import timedelta

from ..test_spatial_state import SPATIAL_STATE_METHOD_EXPECTS
from ...test_state import STATE_METHOD_EXPECTS

CARTESIAN_STATE_METHOD_EXPECTS = [
    'Identity',
    'Random',
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'set_data',
    'dist',
    'get_acceleration',
    'get_angular_acceleration',
    'get_angular_velocity',
    'get_force',
    'get_linear_acceleration',
    'get_linear_velocity',
    'get_name',
    'get_orientation',
    'get_orientation_coefficients',
    'get_pose',
    'get_position',
    'get_reference_frame',
    'get_torque',
    'get_transformation_matrix',
    'get_twist',
    'get_type',
    'get_wrench',
    'reset',
    'inverse',
    'is_incompatible',
    'is_deprecated',
    'is_empty',
    'normalize',
    'normalized',
    'norms',
    'reset_timestamp',
    'set_acceleration',
    'set_angular_acceleration',
    'set_angular_velocity',
    'set_force',
    'set_linear_acceleration',
    'set_linear_velocity',
    'set_name',
    'set_orientation',
    'set_pose',
    'set_position',
    'set_reference_frame',
    'set_torque',
    'set_twist',
    'set_wrench',
    'set_zero',
    'to_list'
]


class TestCartesianState(unittest.TestCase):
    def assert_name_empty_frame_equal(self, state, name, empty, reference_frame):
        self.assertEqual(state.get_name(), name)
        self.assertEqual(state.is_empty(), empty)
        self.assertEqual(state.get_reference_frame(), reference_frame)

    def assert_name_frame_data_equal(self, state1, state2):
        self.assertEqual(state1.get_name(), state2.get_name())
        self.assertEqual(state1.get_reference_frame(), state2.get_reference_frame())
        assert_array_almost_equal(state1.data(), state2.data())

    def clamping_helper(self, state, getter, setter, variable, dim):
        if dim == 3:
            data = [-2.0, 1.0, 5.0]
        elif dim == 6:
            data = [-2.0, 1.0, 5.0, 1.0, -3.0, 2.4]
        else:
            raise ValueError("Provide 3 or 6 as dimension")

        setter(data)
        state.clamp_state_variable(10.0, variable)
        self.assertEqual(type(state), CartesianState)
        [self.assertAlmostEqual(getter()[i], data[i]) for i in range(dim)]
        state.clamp_state_variable(3.0, variable)
        self.assertAlmostEqual(np.linalg.norm(getter()), 3.0)
        state.clamp_state_variable(10.0, variable, 0.5)
        self.assertAlmostEqual(np.linalg.norm(getter()), 0.0)

    def test_callable_methods(self):
        methods = [m for m in dir(CartesianState) if callable(getattr(CartesianState, m))]
        for expected in STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in SPATIAL_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in CARTESIAN_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in CARTESIAN_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        empty1 = CartesianState()
        self.assertTrue(isinstance(empty1, State))
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty1.get_type(), StateType.CARTESIAN_STATE)
        self.assert_name_empty_frame_equal(empty1, "", True, "world")
        self.assertAlmostEqual(np.linalg.norm(empty1.data()), 1)

        empty2 = CartesianState("test")
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty2.get_type(), StateType.CARTESIAN_STATE)
        self.assert_name_empty_frame_equal(empty2, "test", True, "world")
        self.assertAlmostEqual(np.linalg.norm(empty2.data()), 1)

        empty3 = CartesianState("test", "reference")
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty3.get_type(), StateType.CARTESIAN_STATE)
        self.assert_name_empty_frame_equal(empty3, "test", True, "reference")
        self.assertAlmostEqual(np.linalg.norm(empty3.data()), 1)

    def test_identity_initialization(self):
        identity = CartesianState().Identity("test")
        self.assertTrue(isinstance(identity, State))
        self.assertFalse(identity.is_empty())
        self.assertAlmostEqual(np.linalg.norm(identity.get_position()), 0)
        self.assertAlmostEqual(identity.get_orientation().norm, 1)
        self.assertAlmostEqual(identity.get_orientation()[0], 1)
        self.assertAlmostEqual(np.linalg.norm(identity.get_twist()), 0)
        self.assertAlmostEqual(np.linalg.norm(identity.get_acceleration()), 0)
        self.assertAlmostEqual(np.linalg.norm(identity.get_wrench()), 0)

    def test_random_initialization(self):
        random = CartesianState().Random("test")
        self.assertTrue(isinstance(random, State))
        self.assertFalse(random.is_empty())
        self.assertTrue(np.linalg.norm(random.get_position()) > 0)
        self.assertAlmostEqual(random.get_orientation().norm, 1)
        [self.assertTrue(random.get_orientation()[i] != 0) for i in range(4)]
        self.assertTrue(np.linalg.norm(random.get_twist()) > 0)
        self.assertTrue(np.linalg.norm(random.get_acceleration()) > 0)
        self.assertTrue(np.linalg.norm(random.get_wrench()) > 0)

    def test_copy(self):
        state = CartesianState().Random("test")
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assert_name_frame_data_equal(state, state_copy)

    def test_copy_constructor(self):
        random = CartesianState().Random("test")
        copy1 = random
        self.assert_name_frame_data_equal(random, copy1)

        copy2 = random.copy()
        self.assert_name_frame_data_equal(random, copy2)

        copy3 = CartesianState(random)
        self.assert_name_frame_data_equal(random, copy3)

        empty = CartesianState()
        copy4 = empty
        self.assertTrue(copy4.is_empty())
        copy5 = CartesianState(empty)
        self.assertTrue(copy5.is_empty())
        copy6 = empty.copy()
        self.assertTrue(copy6.is_empty())

    def test_get_set_fields(self):
        cs = CartesianState("test")

        # name
        cs.set_name("robot")
        self.assertEqual(cs.get_name(), "robot")
        self.assertEqual(cs.get_reference_frame(), "world")
        cs.set_reference_frame("base")
        self.assertEqual(cs.get_reference_frame(), "base")

        # position
        position = [1., 2., 3.]
        cs.set_position(position)
        [self.assertAlmostEqual(cs.get_position()[i], position[i]) for i in range(3)]
        cs.set_position(1.1, 2.2, 3.3)
        assert_array_equal(np.array([1.1, 2.2, 3.3]), cs.get_position())
        with self.assertRaises(IncompatibleSizeError):
            cs.set_position([1., 2., 3., 4.])

        # orientation coefficients
        orientation_vec = np.random.rand(4)
        orientation_vec = orientation_vec / np.linalg.norm(orientation_vec)
        cs.set_orientation(orientation_vec)
        [self.assertAlmostEqual(cs.get_orientation()[i], orientation_vec[i]) for i in range(4)]
        with self.assertRaises(IncompatibleSizeError):
            cs.set_orientation(orientation_vec[:3].tolist())

        # orientation quaternion
        quaternion = Quaternion.random()
        cs.set_orientation(quaternion)
        assert_array_almost_equal(cs.get_orientation().elements, quaternion.elements)
        with self.assertRaises(ValueError):
            cs.set_orientation(dict())

        matrix = cs.get_transformation_matrix()
        trans = matrix[:3, 3]
        # rot = matrix[:3, :3]
        bottom = matrix[3, :]
        assert_array_almost_equal(trans, cs.get_position())
        # TODO rotation matrix from quaternion
        assert_array_equal(bottom, np.array([0, 0, 0, 1]))

        # pose
        position = [4.4, 5.5, 6.6]
        orientation_vec = np.random.rand(4)
        orientation_vec = orientation_vec / np.linalg.norm(orientation_vec)
        cs.set_pose(np.hstack((position, orientation_vec)))
        assert_array_almost_equal(np.hstack((position, orientation_vec)), cs.get_pose())
        with self.assertRaises(IncompatibleSizeError):
            cs.set_pose(position)

        # twist
        linear_velocity = np.random.rand(3)
        cs.set_linear_velocity(linear_velocity)
        assert_array_almost_equal(cs.get_linear_velocity(), linear_velocity)
        angular_velocity = np.random.rand(3)
        cs.set_angular_velocity(angular_velocity)
        assert_array_almost_equal(cs.get_angular_velocity(), angular_velocity)
        twist = np.random.rand(6)
        cs.set_twist(twist)
        assert_array_almost_equal(cs.get_twist(), twist)

        # acceleration
        linear_acceleration = np.random.rand(3)
        cs.set_linear_acceleration(linear_acceleration)
        assert_array_almost_equal(cs.get_linear_acceleration(), linear_acceleration)
        angular_acceleration = np.random.rand(3)
        cs.set_angular_acceleration(angular_acceleration)
        assert_array_almost_equal(cs.get_angular_acceleration(), angular_acceleration)
        acceleration = np.random.rand(6)
        cs.set_acceleration(acceleration)
        assert_array_almost_equal(cs.get_acceleration(), acceleration)

        # wrench
        force = np.random.rand(3)
        cs.set_force(force)
        assert_array_almost_equal(cs.get_force(), force)
        torque = np.random.rand(3)
        cs.set_torque(torque)
        assert_array_almost_equal(cs.get_torque(), torque)
        wrench = np.random.rand(6)
        cs.set_wrench(wrench)
        assert_array_almost_equal(cs.get_wrench(), wrench)

        cs.set_zero()
        self.assertAlmostEqual(np.linalg.norm(cs.data()), 1)
        self.assertFalse(cs.is_empty())
        cs.reset()
        self.assertTrue(cs.is_empty())

    def test_set_zero(self):
        random1 = CartesianState().Random("test")
        random1.reset()
        self.assertAlmostEqual(np.linalg.norm(random1.data()), 1)

        random2 = CartesianState().Random("test")
        random2.set_zero()
        self.assertAlmostEqual(np.linalg.norm(random1.data()), 1)

    def test_get_set_data(self):
        cs1 = CartesianState().Identity("test")
        cs2 = CartesianState().Random("test")
        concatenated_state = np.hstack((cs1.get_pose(), cs1.get_twist(), cs1.get_acceleration(), cs1.get_wrench()))
        assert_array_almost_equal(cs1.data(), concatenated_state)
        assert_array_almost_equal(cs1.array(), concatenated_state)

        cs1.set_data(cs2.data())
        assert_array_almost_equal(cs1.data(), cs2.data())

        cs2 = CartesianState.Random("test")
        state_vec = cs2.to_list()
        cs1.set_data(state_vec)
        [self.assertAlmostEqual(cs1.data()[i], state_vec[i]) for i in range(len(state_vec))]

        with self.assertRaises(IncompatibleSizeError):
            cs1.set_data(np.array([0, 0]))

    def test_clamping(self):
        state = CartesianState().Identity("test")
        with self.assertRaises(NotImplementedError):
            state.clamp_state_variable(1, CartesianStateVariable.ORIENTATION)
        with self.assertRaises(NotImplementedError):
            state.clamp_state_variable(1, CartesianStateVariable.POSE)
        with self.assertRaises(NotImplementedError):
            state.clamp_state_variable(1, CartesianStateVariable.ALL)

        self.clamping_helper(state, state.get_position, state.set_position, CartesianStateVariable.POSITION, 3)
        self.clamping_helper(state, state.get_linear_velocity, state.set_linear_velocity,
                             CartesianStateVariable.LINEAR_VELOCITY, 3)
        self.clamping_helper(state, state.get_angular_velocity, state.set_angular_velocity,
                             CartesianStateVariable.ANGULAR_VELOCITY, 3)
        self.clamping_helper(state, state.get_twist, state.set_twist, CartesianStateVariable.TWIST, 6)
        self.clamping_helper(state, state.get_linear_acceleration, state.set_linear_acceleration,
                             CartesianStateVariable.LINEAR_ACCELERATION, 3)
        self.clamping_helper(state, state.get_angular_acceleration, state.set_angular_acceleration,
                             CartesianStateVariable.ANGULAR_ACCELERATION, 3)
        self.clamping_helper(state, state.get_acceleration, state.set_acceleration,
                             CartesianStateVariable.ACCELERATION, 6)
        self.clamping_helper(state, state.get_force, state.set_force, CartesianStateVariable.FORCE, 3)
        self.clamping_helper(state, state.get_torque, state.set_torque, CartesianStateVariable.TORQUE, 3)
        self.clamping_helper(state, state.get_wrench, state.set_wrench, CartesianStateVariable.WRENCH, 6)

    def test_norms(self):
        cs = CartesianState().Random("test")
        norms = cs.norms()
        self.assertEqual(len(norms), 8)
        self.assertAlmostEqual(norms[0], np.linalg.norm(cs.get_position()))
        self.assertAlmostEqual(norms[1], cs.get_orientation().norm)
        self.assertAlmostEqual(norms[2], np.linalg.norm(cs.get_linear_velocity()))
        self.assertAlmostEqual(norms[3], np.linalg.norm(cs.get_angular_velocity()))
        self.assertAlmostEqual(norms[4], np.linalg.norm(cs.get_linear_acceleration()))
        self.assertAlmostEqual(norms[5], np.linalg.norm(cs.get_angular_acceleration()))
        self.assertAlmostEqual(norms[6], np.linalg.norm(cs.get_force()))
        self.assertAlmostEqual(norms[7], np.linalg.norm(cs.get_torque()))

    def test_normalize(self):
        cs = CartesianState().Random("test")
        normalized = cs.normalized()
        self.assertEqual(type(normalized), CartesianState)
        norms1 = normalized.norms()
        [self.assertAlmostEqual(n, 1) for n in norms1]

        cs.normalize()
        norms2 = cs.norms()
        [self.assertAlmostEqual(n, 1) for n in norms2]

    def test_distance(self):
        empty = CartesianState()
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test", "robot")

        with self.assertRaises(EmptyStateError):
            empty.dist(cs1)
        with self.assertRaises(EmptyStateError):
            cs1.dist(empty)
        with self.assertRaises(IncompatibleReferenceFramesError):
            cs1.dist(cs2)

        data1 = np.random.rand(25)
        cs1.set_data(data1)
        cs3 = CartesianState("test")
        data3 = np.random.rand(25)
        cs3.set_data(data3)

        pos_dist = np.linalg.norm(data1[:3] - data3[:3])
        # TODO orientation distance
        orient_dist = cs1.dist(cs3, CartesianStateVariable.ORIENTATION)
        lin_vel_dist = np.linalg.norm(data1[7:10] - data3[7:10])
        ang_vel_dist = np.linalg.norm(data1[10:13] - data3[10:13])
        lin_acc_dist = np.linalg.norm(data1[13:16] - data3[13:16])
        ang_acc_dist = np.linalg.norm(data1[16:19] - data3[16:19])
        force_dist = np.linalg.norm(data1[19:22] - data3[19:22])
        torque_dist = np.linalg.norm(data1[22:] - data3[22:])

        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.POSITION), pos_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.ORIENTATION), orient_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.POSE), pos_dist + orient_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.TWIST), lin_vel_dist + ang_vel_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.ACCELERATION), lin_acc_dist + ang_acc_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.WRENCH), force_dist + torque_dist)
        self.assertAlmostEqual(cs1.dist(cs3), cs3.dist(cs1, CartesianStateVariable.ALL))

    def test_addition(self):
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test")
        cs3 = CartesianState().Random("test", "reference")

        with self.assertRaises(IncompatibleReferenceFramesError):
            cs1 + cs3

        csum = cs1 + cs2
        self.assertEqual(type(csum), CartesianState)
        assert_array_almost_equal(csum.get_position(), cs1.get_position() + cs2.get_position())
        # TODO orientation sum
        assert_array_almost_equal(csum.get_twist(), cs1.get_twist() + cs2.get_twist())
        assert_array_almost_equal(csum.get_acceleration(), cs1.get_acceleration() + cs2.get_acceleration())
        assert_array_almost_equal(csum.get_wrench(), cs1.get_wrench() + cs2.get_wrench())

        cs1 += cs2
        self.assertEqual(type(cs1), CartesianState)
        assert_array_almost_equal(cs1.data(), csum.data())

    def test_subtraction(self):
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test")
        cs3 = CartesianState().Random("test", "reference")

        with self.assertRaises(IncompatibleReferenceFramesError):
            cs1 - cs3

        cdiff = cs1 - cs2
        self.assertEqual(type(cdiff), CartesianState)
        assert_array_almost_equal(cdiff.get_position(), cs1.get_position() - cs2.get_position())
        # TODO orientation diff
        assert_array_almost_equal(cdiff.get_twist(), cs1.get_twist() - cs2.get_twist())
        assert_array_almost_equal(cdiff.get_acceleration(), cs1.get_acceleration() - cs2.get_acceleration())
        assert_array_almost_equal(cdiff.get_wrench(), cs1.get_wrench() - cs2.get_wrench())

        cs1 -= cs2
        self.assertEqual(type(cs1), CartesianState)
        assert_array_almost_equal(cs1.data(), cdiff.data())

    def test_scalar_multiplication(self):
        scalar = 2.0
        cs = CartesianState().Random("test")
        cscaled = scalar * cs
        self.assertEqual(type(cscaled), CartesianState)
        assert_array_almost_equal(cscaled.get_position(), scalar * cs.get_position())
        # TODO orientation mult
        assert_array_almost_equal(cscaled.get_twist(), scalar * cs.get_twist())
        assert_array_almost_equal(cscaled.get_acceleration(), scalar * cs.get_acceleration())
        assert_array_almost_equal(cscaled.get_wrench(), scalar * cs.get_wrench())

        cs *= scalar
        self.assertEqual(type(cs), CartesianState)
        assert_array_almost_equal(cs.data(), cscaled.data())

        empty = CartesianState()
        with self.assertRaises(EmptyStateError):
            scalar * empty

    def test_scalar_division(self):
        scalar = 2.0
        cs = CartesianState().Random("test")
        cscaled = cs / scalar
        self.assertEqual(type(cscaled), CartesianState)
        assert_array_almost_equal(cscaled.get_position(), cs.get_position() / scalar)
        # TODO orientation diff
        assert_array_almost_equal(cscaled.get_twist(), cs.get_twist() / scalar)
        assert_array_almost_equal(cscaled.get_acceleration(), cs.get_acceleration() / scalar)
        assert_array_almost_equal(cscaled.get_wrench(), cs.get_wrench() / scalar)

        cs /= scalar
        self.assertEqual(type(cs), CartesianState)
        assert_array_almost_equal(cs.data(), cscaled.data())

        with self.assertRaises(RuntimeError):
            cs / 0.0

        empty = CartesianState()
        with self.assertRaises(EmptyStateError):
            empty / scalar

    def test_truthiness(self):
        empty = CartesianState("test")
        self.assertTrue(empty.is_empty())
        self.assertFalse(empty)

        empty.set_data(CartesianState().Random("test").data())
        self.assertFalse(empty.is_empty())
        self.assertTrue(empty)

    def test_state_multiplication_operators(self):
        state = CartesianState.Random("world")
        pose = CartesianPose.Random("world")
        twist = CartesianTwist.Random("world")
        acceleration = CartesianAcceleration.Random("world")
        wrench = CartesianWrench.Random("world")

        # CartesianState multiplied with any derived stays a CartesianState
        res = state * state
        self.assertIsInstance(res, CartesianState)
        res = state * pose
        self.assertIsInstance(res, CartesianState)
        res = state * twist
        self.assertIsInstance(res, CartesianState)
        res = state * acceleration
        self.assertIsInstance(res, CartesianState)
        res = state * wrench
        self.assertIsInstance(res, CartesianState)

        # CartesianPose multiplied with any derived type is defined by the right hand type
        res = pose * state
        self.assertIsInstance(res, CartesianState)
        res = pose * pose
        self.assertIsInstance(res, CartesianPose)
        res = pose * twist
        self.assertIsInstance(res, CartesianTwist)
        res = pose * acceleration
        self.assertIsInstance(res, CartesianAcceleration)
        res = pose * wrench
        self.assertIsInstance(res, CartesianWrench)

        with self.assertRaises(TypeError):
            res = twist * state
        with self.assertRaises(TypeError):
            res = twist * pose
        with self.assertRaises(TypeError):
            res = twist * twist
        with self.assertRaises(TypeError):
            res = twist * acceleration
        with self.assertRaises(TypeError):
            res = twist * wrench
        with self.assertRaises(TypeError):
            res = acceleration * state
        with self.assertRaises(TypeError):
            res = acceleration * pose
        with self.assertRaises(TypeError):
            res = acceleration * twist
        with self.assertRaises(TypeError):
            res = acceleration * acceleration
        with self.assertRaises(TypeError):
            res = acceleration * wrench
        with self.assertRaises(TypeError):
            res = wrench * state
        with self.assertRaises(TypeError):
            res = wrench * pose
        with self.assertRaises(TypeError):
            res = wrench * twist
        with self.assertRaises(TypeError):
            res = wrench * acceleration
        with self.assertRaises(TypeError):
            res = wrench * wrench

        state *= state
        self.assertIsInstance(state, CartesianState)
        state *= pose
        self.assertIsInstance(state, CartesianState)
        state *= twist
        self.assertIsInstance(state, CartesianState)
        state *= acceleration
        self.assertIsInstance(state, CartesianState)
        state *= wrench
        self.assertIsInstance(state, CartesianState)

        pose *= state
        self.assertIsInstance(pose, CartesianPose)
        pose *= pose
        self.assertIsInstance(pose, CartesianPose)
        with self.assertRaises(TypeError):
            pose *= twist
        with self.assertRaises(TypeError):
            pose *= acceleration
        with self.assertRaises(TypeError):
            pose *= wrench

        with self.assertRaises(TypeError):
            twist *= state
        with self.assertRaises(TypeError):
            twist *= pose
        with self.assertRaises(TypeError):
            twist *= twist
        with self.assertRaises(TypeError):
            twist *= acceleration
        with self.assertRaises(TypeError):
            twist *= wrench

        with self.assertRaises(TypeError):
            acceleration *= state
        with self.assertRaises(TypeError):
            acceleration *= pose
        with self.assertRaises(TypeError):
            acceleration *= twist
        with self.assertRaises(TypeError):
            acceleration *= acceleration
        with self.assertRaises(TypeError):
            acceleration *= wrench

        with self.assertRaises(TypeError):
            wrench *= state
        with self.assertRaises(TypeError):
            wrench *= pose
        with self.assertRaises(TypeError):
            wrench *= twist
        with self.assertRaises(TypeError):
            wrench *= acceleration
        with self.assertRaises(TypeError):
            wrench *= wrench

    def test_state_addition_operators(self):
        state = CartesianState.Random("test")
        pose = CartesianPose.Random("test")
        twist = CartesianTwist.Random("test")
        acceleration = CartesianAcceleration.Random("test")
        wrench = CartesianWrench.Random("test")

        res = pose + pose
        self.assertIsInstance(res, CartesianPose)
        res = state + pose
        self.assertIsInstance(res, CartesianState)
        res = pose + state
        self.assertIsInstance(res, CartesianState)

        res = twist + twist
        self.assertIsInstance(res, CartesianTwist)
        res = state + twist
        self.assertIsInstance(res, CartesianState)
        res = twist + state
        self.assertIsInstance(res, CartesianState)

        res = acceleration + acceleration
        self.assertIsInstance(res, CartesianAcceleration)
        res = state + acceleration
        self.assertIsInstance(res, CartesianState)
        res = acceleration + state
        self.assertIsInstance(res, CartesianState)

        res = wrench + wrench
        self.assertIsInstance(res, CartesianWrench)
        res = state + wrench
        self.assertIsInstance(res, CartesianState)
        res = wrench + state
        self.assertIsInstance(res, CartesianState)

        with self.assertRaises(TypeError):
            res = pose + twist
        with self.assertRaises(TypeError):
            res = pose + acceleration
        with self.assertRaises(TypeError):
            res = pose + wrench

        with self.assertRaises(TypeError):
            res = twist + pose
        with self.assertRaises(TypeError):
            res = twist + acceleration
        with self.assertRaises(TypeError):
            res = twist + wrench

        with self.assertRaises(TypeError):
            res = acceleration + pose
        with self.assertRaises(TypeError):
            res = acceleration + twist
        with self.assertRaises(TypeError):
            res = acceleration + wrench

        with self.assertRaises(TypeError):
            res = wrench + pose
        with self.assertRaises(TypeError):
            res = wrench + twist
        with self.assertRaises(TypeError):
            res = wrench + acceleration

        state += state
        self.assertIsInstance(state, CartesianState)
        state += pose
        self.assertIsInstance(state, CartesianState)
        state += twist
        self.assertIsInstance(state, CartesianState)
        state += acceleration
        self.assertIsInstance(state, CartesianState)
        state += wrench
        self.assertIsInstance(state, CartesianState)

        pose += state
        self.assertIsInstance(pose, CartesianPose)
        pose += pose
        self.assertIsInstance(pose, CartesianPose)
        with self.assertRaises(TypeError):
            pose += twist
        with self.assertRaises(TypeError):
            pose += acceleration
        with self.assertRaises(TypeError):
            pose += wrench

        twist += state
        self.assertIsInstance(twist, CartesianTwist)
        twist += twist
        self.assertIsInstance(twist, CartesianTwist)
        with self.assertRaises(TypeError):
            twist += pose
        with self.assertRaises(TypeError):
            twist += acceleration
        with self.assertRaises(TypeError):
            twist += wrench

        acceleration += state
        self.assertIsInstance(acceleration, CartesianAcceleration)
        acceleration += acceleration
        self.assertIsInstance(acceleration, CartesianAcceleration)
        with self.assertRaises(TypeError):
            acceleration += pose
        with self.assertRaises(TypeError):
            acceleration += twist
        with self.assertRaises(TypeError):
            acceleration += wrench

        wrench += state
        self.assertIsInstance(wrench, CartesianWrench)
        wrench += wrench
        self.assertIsInstance(wrench, CartesianWrench)
        with self.assertRaises(TypeError):
            wrench += pose
        with self.assertRaises(TypeError):
            wrench += twist
        with self.assertRaises(TypeError):
            wrench += acceleration

    def test_state_subtraction_operators(self):
        state = CartesianState.Random("test")
        pose = CartesianPose.Random("test")
        twist = CartesianTwist.Random("test")
        acceleration = CartesianAcceleration.Random("test")
        wrench = CartesianWrench.Random("test")

        res = pose - pose
        self.assertIsInstance(res, CartesianPose)
        res = state - pose
        self.assertIsInstance(res, CartesianState)
        res = pose - state
        self.assertIsInstance(res, CartesianState)

        res = twist - twist
        self.assertIsInstance(res, CartesianTwist)
        res = state - twist
        self.assertIsInstance(res, CartesianState)
        res = twist - state
        self.assertIsInstance(res, CartesianState)

        res = acceleration - acceleration
        self.assertIsInstance(res, CartesianAcceleration)
        res = state - acceleration
        self.assertIsInstance(res, CartesianState)
        res = acceleration - state
        self.assertIsInstance(res, CartesianState)

        res = wrench - wrench
        self.assertIsInstance(res, CartesianWrench)
        res = state - wrench
        self.assertIsInstance(res, CartesianState)
        res = wrench - state
        self.assertIsInstance(res, CartesianState)

        with self.assertRaises(TypeError):
            res = pose - twist
        with self.assertRaises(TypeError):
            res = pose - acceleration
        with self.assertRaises(TypeError):
            res = pose - wrench

        with self.assertRaises(TypeError):
            res = twist - pose
        with self.assertRaises(TypeError):
            res = twist - acceleration
        with self.assertRaises(TypeError):
            res = twist - wrench

        with self.assertRaises(TypeError):
            res = acceleration - pose
        with self.assertRaises(TypeError):
            res = acceleration - twist
        with self.assertRaises(TypeError):
            res = acceleration - wrench

        with self.assertRaises(TypeError):
            res = wrench - pose
        with self.assertRaises(TypeError):
            res = wrench - twist
        with self.assertRaises(TypeError):
            res = wrench - acceleration

        state -= state
        self.assertIsInstance(state, CartesianState)
        state -= pose
        self.assertIsInstance(state, CartesianState)
        state -= twist
        self.assertIsInstance(state, CartesianState)
        state -= acceleration
        self.assertIsInstance(state, CartesianState)
        state -= wrench
        self.assertIsInstance(state, CartesianState)

        pose -= state
        self.assertIsInstance(pose, CartesianPose)
        pose -= pose
        self.assertIsInstance(pose, CartesianPose)
        with self.assertRaises(TypeError):
            pose -= twist
        with self.assertRaises(TypeError):
            pose -= acceleration
        with self.assertRaises(TypeError):
            pose -= wrench

        twist -= state
        self.assertIsInstance(twist, CartesianTwist)
        twist -= twist
        self.assertIsInstance(twist, CartesianTwist)
        with self.assertRaises(TypeError):
            twist -= pose
        with self.assertRaises(TypeError):
            twist -= acceleration
        with self.assertRaises(TypeError):
            twist -= wrench

        acceleration -= state
        self.assertIsInstance(acceleration, CartesianAcceleration)
        acceleration -= acceleration
        self.assertIsInstance(acceleration, CartesianAcceleration)
        with self.assertRaises(TypeError):
            acceleration -= pose
        with self.assertRaises(TypeError):
            acceleration -= twist
        with self.assertRaises(TypeError):
            acceleration -= wrench

        wrench -= state
        self.assertIsInstance(wrench, CartesianWrench)
        wrench -= wrench
        self.assertIsInstance(wrench, CartesianWrench)
        with self.assertRaises(TypeError):
            wrench -= pose
        with self.assertRaises(TypeError):
            wrench -= twist
        with self.assertRaises(TypeError):
            wrench -= acceleration

    def test_multiplication_operators(self):
        state = CartesianState.Random("world")
        pose = CartesianPose.Random("world")
        twist = CartesianTwist.Random("world")
        acceleration = CartesianAcceleration.Random("world")
        wrench = CartesianWrench.Random("world")
        mat = np.random.rand(4, 4)
        comp_mat = np.random.rand(6, 6)

        # state
        state *= 3.0
        self.assertIsInstance(state, CartesianState)
        result = state * 3.0
        self.assertIsInstance(result, CartesianState)
        result = 3.0 * state
        self.assertIsInstance(result, CartesianState)
        arr = np.array([1.1, 2.2, 3.3])
        result = state * np.array([1.1, 2.2, 3.3])
        self.assertIsInstance(result, type(arr))
        self.assertTrue(len(result) == 3)
        with self.assertRaises(TypeError):
            state *= mat
        with self.assertRaises(TypeError):
            result = state * mat
        with self.assertRaises(TypeError):
            result = mat * state
        with self.assertRaises(TypeError):
            result = state / mat
        with self.assertRaises(TypeError):
            result = mat / state
        with self.assertRaises(TypeError):
            state /= mat
        state /= 2.0
        self.assertIsInstance(state, CartesianState)
        result = state / 2.0
        self.assertIsInstance(result, CartesianState)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * state
        with self.assertRaises(TypeError):
            state / timedelta(seconds=1)

        # pose
        pose *= 3.0
        self.assertIsInstance(pose, CartesianPose)
        result = pose * 3.0
        self.assertIsInstance(result, CartesianPose)
        result = 3.0 * pose
        self.assertIsInstance(result, CartesianPose)
        with self.assertRaises(TypeError):
            result = pose * mat
        with self.assertRaises(TypeError):
            result = mat * pose
        with self.assertRaises(TypeError):
            pose *= mat
        with self.assertRaises(TypeError):
            result = pose / mat
        with self.assertRaises(TypeError):
            result = mat / pose
        with self.assertRaises(TypeError):
            pose /= mat
        result = pose / 2.0
        self.assertIsInstance(result, CartesianPose)
        pose /= 2.0
        self.assertIsInstance(pose, CartesianPose)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * pose
        result = pose / timedelta(seconds=1)
        self.assertIsInstance(result, CartesianTwist)

        # twist
        twist *= 3.0
        self.assertIsInstance(twist, CartesianTwist)
        result = twist * 3.0
        self.assertIsInstance(result, CartesianTwist)
        result = 3.0 * twist
        self.assertIsInstance(result, CartesianTwist)
        result = comp_mat * twist
        self.assertIsInstance(result, CartesianTwist)
        with self.assertRaises(TypeError):
            twist *= comp_mat
        with self.assertRaises(TypeError):
            result = twist * comp_mat
        with self.assertRaises(TypeError):
            result = twist * mat
        with self.assertRaises(TypeError):
            result = mat * twist
        with self.assertRaises(TypeError):
            twist *= mat
        with self.assertRaises(TypeError):
            result = twist / mat
        with self.assertRaises(TypeError):
            result = mat / twist
        with self.assertRaises(TypeError):
            twist /= mat
        twist /= 3.0
        self.assertIsInstance(twist, CartesianTwist)
        result = twist / 3.0
        self.assertIsInstance(result, CartesianTwist)
        result = twist * timedelta(seconds=1)
        self.assertIsInstance(result, CartesianPose)
        result = timedelta(seconds=1) * twist
        self.assertIsInstance(result, CartesianPose)
        result = twist / timedelta(seconds=1)
        self.assertIsInstance(result, CartesianAcceleration)

        # acceleration
        acceleration *= 3.0
        self.assertIsInstance(acceleration, CartesianAcceleration)
        result = acceleration * 3.0
        self.assertIsInstance(result, CartesianAcceleration)
        result = 3.0 * acceleration
        self.assertIsInstance(result, CartesianAcceleration)
        result = comp_mat * acceleration
        self.assertIsInstance(result, CartesianAcceleration)
        with self.assertRaises(TypeError):
            acceleration *= comp_mat
        with self.assertRaises(TypeError):
            result = acceleration * comp_mat
        with self.assertRaises(TypeError):
            result = acceleration * mat
        with self.assertRaises(TypeError):
            result = mat * acceleration
        with self.assertRaises(TypeError):
            acceleration *= mat
        with self.assertRaises(TypeError):
            result = acceleration / mat
        with self.assertRaises(TypeError):
            result = mat / acceleration
        with self.assertRaises(TypeError):
            acceleration /= mat
        acceleration /= 3.0
        self.assertIsInstance(acceleration, CartesianAcceleration)
        result = acceleration / 3.0
        self.assertIsInstance(result, CartesianAcceleration)
        result = acceleration * timedelta(seconds=1)
        self.assertIsInstance(result, CartesianTwist)
        result = timedelta(seconds=1) * acceleration
        self.assertIsInstance(result, CartesianTwist)
        with self.assertRaises(TypeError):
            acceleration / timedelta(seconds=1)

        # wrench
        wrench *= 3.0
        self.assertIsInstance(wrench, CartesianWrench)
        result = wrench * 3.0
        self.assertIsInstance(result, CartesianWrench)
        result = 3.0 * wrench
        self.assertIsInstance(result, CartesianWrench)
        result = comp_mat * wrench
        self.assertIsInstance(result, CartesianWrench)
        with self.assertRaises(TypeError):
            wrench *= comp_mat
        with self.assertRaises(TypeError):
            result = wrench * comp_mat
        with self.assertRaises(TypeError):
            result = acceleration * mat
        with self.assertRaises(TypeError):
            result = mat * acceleration
        with self.assertRaises(TypeError):
            acceleration *= mat
        with self.assertRaises(TypeError):
            result = acceleration / mat
        with self.assertRaises(TypeError):
            result = mat / acceleration
        with self.assertRaises(TypeError):
            acceleration /= mat
        wrench /= 3.0
        self.assertIsInstance(wrench, CartesianWrench)
        result = wrench / 3.0
        self.assertIsInstance(result, CartesianWrench)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * wrench
        with self.assertRaises(TypeError):
            wrench / timedelta(seconds=1)


if __name__ == '__main__':
    unittest.main()
