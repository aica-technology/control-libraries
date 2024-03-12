import os
import unittest

from robot_model import Model
from state_representation import JointState

class RobotModelCollisionTesting(unittest.TestCase):
    ur5e_with_geometries = None
    ur5e_without_geometries = None
    test_non_colliding_configs = []
    test_colliding_configs = []

    @classmethod
    def setUpClass(cls):
        package_paths = [os.path.join(os.path.dirname(os.path.realpath(__file__)), "ur5e")]
        cls.ur5e_with_geometries = Model("ur5e", os.path.join(package_paths[0], "ur5e.urdf"), package_paths)
        cls.ur5e_without_geometries = Model("ur5e", os.path.join(package_paths[0], "ur5e.urdf"))
        cls.set_test_non_colliding_configurations()
        cls.set_test_colliding_configurations()

    @classmethod
    def set_test_non_colliding_configurations(cls):
        config1 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config1.set_positions([0.0, -1.63, 1.45, 0.38, 0.0, 0.0])
        cls.test_non_colliding_configs.append(config1)

        config2 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config2.set_positions([0.0, -1.88, 1.45, 0.38, -4.4, -3.14])
        cls.test_non_colliding_configs.append(config2)

        config3 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config3.set_positions([1.26, -1.26, 0.82, 0.38, -4.4, 3.14])
        cls.test_non_colliding_configs.append(config3)

    @classmethod
    def set_test_colliding_configurations(cls):
        config1 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config1.set_positions([1.26, -1.76, 2.89, 0.38, -4.4, -6.16])
        cls.test_colliding_configs.append(config1)

        config2 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config2.set_positions([1.26, -1.76, 2.89, 0.38, -1.38, -1.16])
        cls.test_colliding_configs.append(config2)

        config3 = JointState(cls.ur5e_with_geometries.get_robot_name(), 6)
        config3.set_positions([1.26, -1.76, -3.08, 0.75, -1.38, -6.16])
        cls.test_colliding_configs.append(config3)

    def test_number_of_collision_pairs_with_geometries(self):
        num_pairs = self.ur5e_with_geometries.get_number_of_collision_pairs()
        self.assertGreater(num_pairs, 6, "Expected more than 6 collision pairs for ur5e with geometries.")

    def test_number_of_collision_pairs_without_geometries(self):
        num_pairs = self.ur5e_without_geometries.get_number_of_collision_pairs()
        self.assertEqual(num_pairs, 0, "Expected zero collision pairs for model without geometries.")

    def test_geom_model_initialized_with_geometries(self):
        is_initialized = self.ur5e_with_geometries.is_geometry_model_initialized()
        self.assertTrue(is_initialized, "Expected geometry model to be initialized for model with geometries.")

    def test_geom_model_initialized_without_geometries(self):
        is_initialized = self.ur5e_without_geometries.is_geometry_model_initialized()
        self.assertFalse(is_initialized, "Expected geometry model to not be initialized for model without geometries.")

    def test_no_collision_detected(self):
        for config in self.test_non_colliding_configs:
            is_colliding = self.ur5e_with_geometries.check_collision(config)
            self.assertFalse(is_colliding, "Expected no collision for configuration")

    def test_collision_detected(self):
        for config in self.test_colliding_configs:
            is_colliding = self.ur5e_with_geometries.check_collision(config)
            self.assertTrue(is_colliding, "Expected collision for configuration")


if __name__ == '__main__':
    unittest.main()
