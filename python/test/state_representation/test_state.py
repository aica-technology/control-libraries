import time
import unittest
import datetime
import copy

from state_representation import State, StateType

STATE_METHOD_EXPECTS = [
    'get_type',
    'is_empty',
    'get_age',
    'get_timestamp',
    'reset_timestamp',
    'get_name',
    'set_name',
    'is_deprecated',
    'is_incompatible',
    'reset'
]


class TestState(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(State) if callable(getattr(State, m))]
        for expected in STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        empty1 = State()
        self.assertEqual(empty1.get_type(), StateType.STATE)
        self.assertEqual(empty1.get_name(), "")
        self.assertTrue(empty1.is_empty())

        empty2 = State("test")
        self.assertEqual(empty2.get_type(), StateType.STATE)
        self.assertEqual(empty2.get_name(), "test")
        self.assertTrue(empty2.is_empty())

        state = State(empty2)
        self.assertEqual(state.get_type(), StateType.STATE)
        self.assertEqual(state.get_name(), "test")
        self.assertTrue(state.is_empty())

    def test_compatibility(self):
        state1 = State()
        state1.set_name("test")
        self.assertEqual(state1.get_name(), "test")

        state2 = State("test")
        self.assertFalse(state1.is_incompatible(state2))

        state2.reset()
        self.assertTrue(state2.is_empty())

    def test_timestamp(self):
        state = State("test")
        time.sleep(0.2)
        self.assertTrue(state.is_deprecated(datetime.timedelta(milliseconds=100)))
        self.assertTrue(state.is_deprecated(0.1))
        state.reset_timestamp()
        self.assertFalse(state.is_deprecated(datetime.timedelta(milliseconds=100)))
        self.assertFalse(state.is_deprecated(0.1))
        self.assertTrue(state.get_age() < 0.1)
        time.sleep(0.2)
        self.assertTrue(state.is_deprecated(datetime.timedelta(milliseconds=100)))
        self.assertTrue(state.is_deprecated(0.1))
        state.reset_timestamp()
        self.assertFalse(state.is_deprecated(datetime.timedelta(milliseconds=100)))
        self.assertFalse(state.is_deprecated(0.1))
        time.sleep(0.2)
        self.assertTrue(state.get_age() > 0.2)

    def test_copy(self):
        state = State("test")
        state2 = copy.copy(state)
        state3 = copy.deepcopy(state)

    def test_truthiness(self):
        state = State()
        self.assertTrue(state.is_empty())
        self.assertFalse(state)

if __name__ == '__main__':
    unittest.main()
