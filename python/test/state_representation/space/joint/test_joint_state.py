import unittest
import copy

import numpy as np
from state_representation import JointState, JointPositions, JointVelocities, JointAccelerations, JointTorques, \
    JointStateVariable, string_to_joint_state_variable, joint_state_variable_to_string
from state_representation.exceptions import InvalidStateVariableError, IncompatibleSizeError
from datetime import timedelta

JOINT_STATE_METHOD_EXPECTS = [
    'Random',
    'Zero',
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'set_data',
    'dist',
    'get_accelerations',
    'get_acceleration',
    'get_joint_index',
    'get_name',
    'get_names',
    'get_positions',
    'get_position',
    'get_size',
    'get_torques',
    'get_torque',
    'get_type',
    'get_velocities',
    'get_velocity',
    'reset',
    'is_incompatible',
    'is_deprecated',
    'is_empty',
    'reset_timestamp',
    'set_accelerations',
    'set_name',
    'set_names',
    'set_positions',
    'set_position',
    'set_velocities',
    'set_velocity',
    'set_accelerations',
    'set_acceleration',
    'set_torques',
    'set_torque',
    'set_zero',
    'to_list',
    'multiply_state_variable',
    'get_state_variable',
    'set_state_variable'
]


class TestJointState(unittest.TestCase):
    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_callable_methods(self):
        methods = [m for m in dir(JointState) if callable(getattr(JointState, m))]
        for expected in JOINT_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_copy(self):
        state = JointState().Random("test", 3)
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_name(), state_copy.get_name())
            self.assertListEqual(state.get_names(), state_copy.get_names())
            self.assert_np_array_equal(state.data(), state_copy.data())

    def test_truthiness(self):
        empty = JointState("test", 3)
        self.assertTrue(empty.is_empty())
        self.assertFalse(empty)

        empty.set_data(JointState().Random("test", 3).data())
        self.assertFalse(empty.is_empty())
        self.assertTrue(empty)

    def test_state_addition_operators(self):
        state = JointState().Random("test", 3)
        positions = JointPositions().Random("test", 3)
        velocities = JointVelocities().Random("test", 3)
        accelerations = JointAccelerations().Random("test", 3)
        torques = JointTorques().Random("test", 3)

        res = positions + positions
        self.assertIsInstance(res, JointPositions)
        res = state + positions
        self.assertIsInstance(res, JointState)
        res = positions + state
        self.assertIsInstance(res, JointState)

        res = velocities + velocities
        self.assertIsInstance(res, JointVelocities)
        res = state + velocities
        self.assertIsInstance(res, JointState)
        res = velocities + state
        self.assertIsInstance(res, JointState)

        res = accelerations + accelerations
        self.assertIsInstance(res, JointAccelerations)
        res = state + accelerations
        self.assertIsInstance(res, JointState)
        res = accelerations + state
        self.assertIsInstance(res, JointState)

        res = torques + torques
        self.assertIsInstance(res, JointTorques)
        res = state + torques
        self.assertIsInstance(res, JointState)
        res = torques + state
        self.assertIsInstance(res, JointState)

        with self.assertRaises(TypeError):
            res = positions + velocities
        with self.assertRaises(TypeError):
            res = positions + accelerations
        with self.assertRaises(TypeError):
            res = positions + torques

        with self.assertRaises(TypeError):
            res = velocities + positions
        with self.assertRaises(TypeError):
            res = velocities + accelerations
        with self.assertRaises(TypeError):
            res = velocities + torques

        with self.assertRaises(TypeError):
            res = accelerations + positions
        with self.assertRaises(TypeError):
            res = accelerations + velocities
        with self.assertRaises(TypeError):
            res = accelerations + torques

        with self.assertRaises(TypeError):
            res = torques + positions
        with self.assertRaises(TypeError):
            res = torques + velocities
        with self.assertRaises(TypeError):
            res = torques + accelerations

        state += state
        self.assertIsInstance(state, JointState)
        state += positions
        self.assertIsInstance(state, JointState)
        state += velocities
        self.assertIsInstance(state, JointState)
        state += accelerations
        self.assertIsInstance(state, JointState)
        state += torques
        self.assertIsInstance(state, JointState)

        positions += state
        self.assertIsInstance(positions, JointPositions)
        positions += positions
        self.assertIsInstance(positions, JointPositions)
        with self.assertRaises(TypeError):
            positions += velocities
        with self.assertRaises(TypeError):
            positions += accelerations
        with self.assertRaises(TypeError):
            positions += torques

        velocities += state
        self.assertIsInstance(velocities, JointVelocities)
        velocities += velocities
        self.assertIsInstance(velocities, JointVelocities)
        with self.assertRaises(TypeError):
            velocities += positions
        with self.assertRaises(TypeError):
            velocities += accelerations
        with self.assertRaises(TypeError):
            velocities += torques

        accelerations += state
        self.assertIsInstance(accelerations, JointAccelerations)
        accelerations += accelerations
        self.assertIsInstance(accelerations, JointAccelerations)
        with self.assertRaises(TypeError):
            accelerations += positions
        with self.assertRaises(TypeError):
            accelerations += velocities
        with self.assertRaises(TypeError):
            accelerations += torques

        torques += state
        self.assertIsInstance(torques, JointTorques)
        torques += torques
        self.assertIsInstance(torques, JointTorques)
        with self.assertRaises(TypeError):
            torques += positions
        with self.assertRaises(TypeError):
            torques += velocities
        with self.assertRaises(TypeError):
            torques += accelerations

    def test_state_subtraction_operators(self):
        state = JointState().Random("test", 3)
        positions = JointPositions().Random("test", 3)
        velocities = JointVelocities().Random("test", 3)
        accelerations = JointAccelerations().Random("test", 3)
        torques = JointTorques().Random("test", 3)

        res = positions - positions
        self.assertIsInstance(res, JointPositions)
        res = state - positions
        self.assertIsInstance(res, JointState)
        res = positions - state
        self.assertIsInstance(res, JointState)

        res = velocities - velocities
        self.assertIsInstance(res, JointVelocities)
        res = state - velocities
        self.assertIsInstance(res, JointState)
        res = velocities - state
        self.assertIsInstance(res, JointState)

        res = accelerations - accelerations
        self.assertIsInstance(res, JointAccelerations)
        res = state - accelerations
        self.assertIsInstance(res, JointState)
        res = accelerations - state
        self.assertIsInstance(res, JointState)

        res = torques - torques
        self.assertIsInstance(res, JointTorques)
        res = state - torques
        self.assertIsInstance(res, JointState)
        res = torques - state
        self.assertIsInstance(res, JointState)

        with self.assertRaises(TypeError):
            res = positions - velocities
        with self.assertRaises(TypeError):
            res = positions - accelerations
        with self.assertRaises(TypeError):
            res = positions - torques

        with self.assertRaises(TypeError):
            res = velocities - positions
        with self.assertRaises(TypeError):
            res = velocities - accelerations
        with self.assertRaises(TypeError):
            res = velocities - torques

        with self.assertRaises(TypeError):
            res = accelerations - positions
        with self.assertRaises(TypeError):
            res = accelerations - velocities
        with self.assertRaises(TypeError):
            res = accelerations - torques

        with self.assertRaises(TypeError):
            res = torques - positions
        with self.assertRaises(TypeError):
            res = torques - velocities
        with self.assertRaises(TypeError):
            res = torques - accelerations

        state -= state
        self.assertIsInstance(state, JointState)
        state -= positions
        self.assertIsInstance(state, JointState)
        state -= velocities
        self.assertIsInstance(state, JointState)
        state -= accelerations
        self.assertIsInstance(state, JointState)
        state -= torques
        self.assertIsInstance(state, JointState)

        positions -= state
        self.assertIsInstance(positions, JointPositions)
        positions -= positions
        self.assertIsInstance(positions, JointPositions)
        with self.assertRaises(TypeError):
            positions -= velocities
        with self.assertRaises(TypeError):
            positions -= accelerations
        with self.assertRaises(TypeError):
            positions -= torques

        velocities -= state
        self.assertIsInstance(velocities, JointVelocities)
        velocities -= velocities
        self.assertIsInstance(velocities, JointVelocities)
        with self.assertRaises(TypeError):
            velocities -= positions
        with self.assertRaises(TypeError):
            velocities -= accelerations
        with self.assertRaises(TypeError):
            velocities -= torques

        accelerations -= state
        self.assertIsInstance(accelerations, JointAccelerations)
        accelerations -= accelerations
        self.assertIsInstance(accelerations, JointAccelerations)
        with self.assertRaises(TypeError):
            accelerations -= positions
        with self.assertRaises(TypeError):
            accelerations -= velocities
        with self.assertRaises(TypeError):
            accelerations -= torques

        torques -= state
        self.assertIsInstance(torques, JointTorques)
        torques -= torques
        self.assertIsInstance(torques, JointTorques)
        with self.assertRaises(TypeError):
            torques -= positions
        with self.assertRaises(TypeError):
            torques -= velocities
        with self.assertRaises(TypeError):
            torques -= accelerations

    def test_multiplication_operators(self):
        state = JointState().Random("test", 3)
        positions = JointPositions().Random("test", 3)
        velocities = JointVelocities().Random("test", 3)
        accelerations = JointAccelerations().Random("test", 3)
        torques = JointTorques().Random("test", 3)

        mat = np.random.rand(12, 12)
        # state
        state *= 3.0
        self.assertIsInstance(state, JointState)
        result = state * 3.0
        self.assertIsInstance(result, JointState)
        result = 3.0 * state
        self.assertIsInstance(result, JointState)
        result = mat * state
        self.assertIsInstance(result, JointState)
        with self.assertRaises(TypeError):
            state *= mat
        with self.assertRaises(TypeError):
            result = state * mat
        with self.assertRaises(TypeError):
            result = state / mat
        with self.assertRaises(TypeError):
            result = mat / state
        with self.assertRaises(TypeError):
            state /= mat
        state /= 2.0
        self.assertIsInstance(state, JointState)
        result = state / 2.0
        self.assertIsInstance(result, JointState)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * state
        with self.assertRaises(TypeError):
            state / timedelta(seconds=1)

        mat = np.random.rand(3, 3)
        # positions
        positions *= 3.0
        self.assertIsInstance(positions, JointPositions)
        result = positions * 3.0
        self.assertIsInstance(result, JointPositions)
        result = 3.0 * positions
        self.assertIsInstance(result, JointPositions)
        result = mat * positions
        self.assertIsInstance(result, JointPositions)
        with self.assertRaises(TypeError):
            positions *= mat
        with self.assertRaises(TypeError):
            result = positions * mat
        with self.assertRaises(TypeError):
            result = positions / mat
        with self.assertRaises(TypeError):
            result = mat / positions
        with self.assertRaises(TypeError):
            positions /= mat
        positions /= 2.0
        self.assertIsInstance(positions, JointPositions)
        result = positions / 2.0
        self.assertIsInstance(result, JointPositions)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * positions
        result = positions / timedelta(seconds=1)
        self.assertIsInstance(result, JointVelocities)

        # velocities
        velocities *= 3.0
        self.assertIsInstance(velocities, JointVelocities)
        result = velocities * 3.0
        self.assertIsInstance(result, JointVelocities)
        result = 3.0 * velocities
        self.assertIsInstance(result, JointVelocities)
        result = mat * velocities
        self.assertIsInstance(result, JointVelocities)
        with self.assertRaises(TypeError):
            velocities *= mat
        with self.assertRaises(TypeError):
            result = velocities * mat
        with self.assertRaises(TypeError):
            result = velocities / mat
        with self.assertRaises(TypeError):
            result = mat / velocities
        with self.assertRaises(TypeError):
            velocities /= mat
        velocities /= 2.0
        self.assertIsInstance(velocities, JointVelocities)
        result = velocities / 2.0
        self.assertIsInstance(result, JointVelocities)
        result = velocities * timedelta(seconds=1)
        self.assertIsInstance(result, JointPositions)
        result = timedelta(seconds=1) * velocities
        self.assertIsInstance(result, JointPositions)
        result = velocities / timedelta(seconds=1)
        self.assertIsInstance(result, JointAccelerations)

        # accelerations
        accelerations *= 3.0
        self.assertIsInstance(accelerations, JointAccelerations)
        result = accelerations * 3.0
        self.assertIsInstance(result, JointAccelerations)
        result = 3.0 * accelerations
        self.assertIsInstance(result, JointAccelerations)
        result = mat * accelerations
        self.assertIsInstance(result, JointAccelerations)
        with self.assertRaises(TypeError):
            accelerations *= mat
        with self.assertRaises(TypeError):
            result = accelerations * mat
        with self.assertRaises(TypeError):
            result = accelerations / mat
        with self.assertRaises(TypeError):
            result = mat / accelerations
        with self.assertRaises(TypeError):
            accelerations /= mat
        accelerations /= 2.0
        self.assertIsInstance(accelerations, JointAccelerations)
        result = accelerations / 2.0
        self.assertIsInstance(result, JointAccelerations)
        result = accelerations * timedelta(seconds=1)
        self.assertIsInstance(result, JointVelocities)
        result = timedelta(seconds=1) * accelerations
        self.assertIsInstance(result, JointVelocities)
        with self.assertRaises(TypeError):
            accelerations / timedelta(seconds=1)

        # torques
        torques *= 3.0
        self.assertIsInstance(torques, JointTorques)
        result = torques * 3.0
        self.assertIsInstance(result, JointTorques)
        result = 3.0 * torques
        self.assertIsInstance(result, JointTorques)
        result = mat * torques
        self.assertIsInstance(result, JointTorques)
        with self.assertRaises(TypeError):
            torques *= mat
        with self.assertRaises(TypeError):
            result = torques * mat
        with self.assertRaises(TypeError):
            result = torques / mat
        with self.assertRaises(TypeError):
            result = mat / torques
        with self.assertRaises(TypeError):
            torques /= mat
        torques /= 2.0
        self.assertIsInstance(torques, JointTorques)
        result = torques / 2.0
        self.assertIsInstance(result, JointTorques)
        with self.assertRaises(TypeError):
            timedelta(seconds=1) * torques
        with self.assertRaises(TypeError):
            torques / timedelta(seconds=1)

    def test_utilities(self):
        state_variable_type = string_to_joint_state_variable("positions")
        self.assertIsInstance(state_variable_type, JointStateVariable)
        self.assertEqual("positions", joint_state_variable_to_string(state_variable_type))
        with self.assertRaises(InvalidStateVariableError):
            string_to_joint_state_variable("foo")

        state = JointState("foo", 3)
        with self.assertRaises(IncompatibleSizeError):
            state.set_state_variable([1.0, 2.0, 3.0, 4.0], JointStateVariable.POSITIONS)
        state.set_state_variable([1.0, 2.0, 3.0], JointStateVariable.POSITIONS)
        self.assert_np_array_equal(state.get_state_variable(JointStateVariable.POSITIONS), [1.0, 2.0, 3.0])
        self.assert_np_array_equal(state.get_state_variable(state_variable_type), [1.0, 2.0, 3.0])

        matrix = np.random.rand(3, 3)
        expected = matrix @ np.array([1.0, 2.0, 3.0])
        state.multiply_state_variable(matrix, JointStateVariable.POSITIONS)
        self.assert_np_array_equal(state.get_positions(), expected)

        state.set_state_variable([4.0, 5.0, 6.0], JointStateVariable.POSITIONS)
        self.assert_np_array_equal(state.get_state_variable(JointStateVariable.POSITIONS), [4.0, 5.0, 6.0])

        state.set_state_variable(np.array([7.0, 8.0, 9.0]), JointStateVariable.POSITIONS)
        self.assert_np_array_equal(state.get_positions(), [7.0, 8.0, 9.0])

if __name__ == '__main__':
    unittest.main()
