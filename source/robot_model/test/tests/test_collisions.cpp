#include "robot_model/Model.hpp"
#include "robot_model/exceptions/CollisionGeometryException.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

using namespace robot_model;

class RobotModelCollisionTesting : public testing::Test {
protected:
  void SetUp() override {
    // create a callback function to get that takes as input a package name and returns the path to the package
    auto package_paths = [](const std::string& package_name) -> std::string {
      if (package_name == "ur_description") {
        return std::string(TEST_FIXTURES);
      } else {
        return "";
      }
    };

    ur5e_without_geometries = std::make_unique<Model>("ur5e", std::string(TEST_FIXTURES) + "ur5e.urdf");
    ur5e_with_geometries = std::make_unique<Model>("ur5e", std::string(TEST_FIXTURES) + "ur5e.urdf", package_paths);
  };

  std::unique_ptr<Model> ur5e_with_geometries;
  std::unique_ptr<Model> ur5e_without_geometries;
  std::vector<state_representation::JointPositions> test_non_coliding_configs;
  std::vector<state_representation::JointPositions> test_coliding_configs;

  // create custom test fixture for robot model without collision
  void set_test_non_coliding_configurations() {
    // Random test configuration 1:
    state_representation::JointPositions config1(ur5e_with_geometries->get_robot_name(), 6);
    config1.set_positions(std::vector<double>{0.0, -1.63, 1.45, 0.38, 0.0, 0.0});
    test_non_coliding_configs.push_back(config1);

    // Random test configuration 2:
    state_representation::JointPositions config2(ur5e_with_geometries->get_robot_name(), 6);
    config2.set_positions(std::vector<double>{0.0, -1.88, 1.45, 0.38, -4.4, -3.14});
    test_non_coliding_configs.push_back(config2);

    // Random test configuration 3:
    state_representation::JointPositions config3(ur5e_with_geometries->get_robot_name(), 6);
    config3.set_positions(std::vector<double>{1.26, -1.26, 0.82, 0.38, -4.4, 3.14});
    test_non_coliding_configs.push_back(config3);
  };

  // create custom test fixture for robot model with collision geometries
  void set_test_coliding_configurations() {
    // Random test configuration 1:
    state_representation::JointPositions config1(ur5e_with_geometries->get_robot_name(), 6);
    config1.set_positions(std::vector<double>{1.26, -1.76, 2.89, 0.38, -4.4, -6.16});
    test_coliding_configs.push_back(config1);

    // Random test configuration 2:
    state_representation::JointPositions config2(ur5e_with_geometries->get_robot_name(), 6);
    config2.set_positions(std::vector<double>{1.26, -1.76, 2.89, 0.38, -1.38, -1.16});
    test_coliding_configs.push_back(config2);

    // Random test configuration 3:
    state_representation::JointPositions config3(ur5e_with_geometries->get_robot_name(), 6);
    config3.set_positions(std::vector<double>{1.26, -1.76, -3.08, 0.75, -1.38, -6.16});
    test_coliding_configs.push_back(config3);
  };
};

// Test calling check_collision() with an uninitialized geometry model
TEST_F(RobotModelCollisionTesting, CheckCollisionWithoutGeometries) {
  // Random test configuration:
  state_representation::JointPositions config(ur5e_without_geometries->get_robot_name(), 6);
  config.set_positions(std::vector<double>{0.0, -1.63, 1.45, 0.38, 0.0, 0.0});
  EXPECT_THROW(ur5e_without_geometries->check_collision(config), exceptions::CollisionGeometryException)
      << "Expected exception for model without geometries.";
}

// Test that get_number_of_collision_pairs() returns 0 for a model without collision geometries loaded
TEST_F(RobotModelCollisionTesting, NumberOfCollisionPairsWithoutGeometries) {
  unsigned int num_pairs = ur5e_without_geometries->get_number_of_collision_pairs();
  EXPECT_EQ(num_pairs, 0) << "Expected zero collision pairs for model without geometries.";
}

// Test that get_number_of_collision_pairs() returns a non-zero value for a model with collision geometries loaded
TEST_F(RobotModelCollisionTesting, NumberOfCollisionPairsWithGeometries) {
  // Assuming your model initialization actually loads collision geometries if available
  unsigned num_pairs = ur5e_with_geometries->get_number_of_collision_pairs();
  EXPECT_EQ(num_pairs, 15) << "Expected 15 collision pairs for ur5e with geometries.";
}

// Test that is_geometry_model_initialized() returns true for a model with collision geometries loaded
TEST_F(RobotModelCollisionTesting, GeomModelInitializedWithGeometries) {
  bool is_initialized = ur5e_with_geometries->is_geometry_model_initialized();
  EXPECT_TRUE(is_initialized) << "Expected geometry model to be initialized for model with geometries.";
}

// Test that is_geometry_model_initialized() returns false for a model without collision geometries loaded
TEST_F(RobotModelCollisionTesting, GeomModelInitializedWithoutGeometries) {
  bool is_initialized = ur5e_without_geometries->is_geometry_model_initialized();
  EXPECT_FALSE(is_initialized) << "Expected geometry model to not be initialized for model without geometries.";
}

// Test that check_collision correctly identifies a collision-free state
TEST_F(RobotModelCollisionTesting, NoCollisionDetected) {
  // iterate over test configurations and check for collision
  set_test_non_coliding_configurations();
  for (auto& config : test_non_coliding_configs) {
    bool is_colliding = ur5e_with_geometries->check_collision(config);
    EXPECT_FALSE(is_colliding) << "Expected no collision for configuration " << config;
  }
}

// Test that check_collision correctly identifies a colliding state
TEST_F(RobotModelCollisionTesting, CollisionDetected) {
  // iterate over test configurations and check for collision
  set_test_coliding_configurations();
  for (auto& config : test_coliding_configs) {
    bool is_colliding = ur5e_with_geometries->check_collision(config);
    EXPECT_TRUE(is_colliding) << "Expected collision for configuration " << config;
  }
}

// Test that compute_minimum_collision_distances method identifies a collision-free state
TEST_F(RobotModelCollisionTesting, MinimumDistanceComputedNoCollision) {
  // iterate over test configurations and check for collision
  set_test_non_coliding_configurations();

  for (auto& config : test_non_coliding_configs) {
    Eigen::MatrixXd distances = ur5e_with_geometries->compute_minimum_collision_distances(config);

    // check that no element is equal to zero besided the diagonals
    EXPECT_EQ(distances.rows(), 6) << "Distance matrix has incorrect number of rows.";
    EXPECT_EQ(distances.cols(), 6) << "Distance matrix has incorrect number of columns.";

    // Then check that no element is equal to zero besides the diagonals
    for (int i = 0; i < distances.rows(); ++i) {
      for (int j = 0; j < distances.cols(); ++j) {
        if (i != j && j != i + 1 && i != j + 1) {// Skip diagonal elements & adjacent links
          EXPECT_GE(distances(i, j), 0.01)
              << "Found a distance at non-diagonal element [" << i << ", " << j << "], indicating a collision.";
        }
      }
    }
  }
}

TEST_F(RobotModelCollisionTesting, MinimumDistanceComputedCollision) {
  // Iterate over test configurations expected to result in collisions
  set_test_coliding_configurations();

  for (auto& config : test_coliding_configs) {
    Eigen::MatrixXd distances = ur5e_with_geometries->compute_minimum_collision_distances(config);

    // Check the size of the distance matrix is 6x6
    EXPECT_EQ(distances.rows(), 6) << "Distance matrix has incorrect number of rows.";
    EXPECT_EQ(distances.cols(), 6) << "Distance matrix has incorrect number of columns.";

    // Initialize a variable to keep track of the minimum non-diagonal distance
    double minimum_distance = std::numeric_limits<double>::max();

    // Iterate over the matrix to find the minimum non-diagonal distance
    for (int i = 0; i < distances.rows(); ++i) {
      for (int j = 0; j < distances.cols(); ++j) {
        if (i != j && j != i + 1 && i != j + 1
            && distances(i, j) < minimum_distance) {// Skip diagonal elements & adjacent links
          minimum_distance = distances(i, j);
        }
      }
    }

    // Expect the minimum non-diagonal distance to be 0, indicating a collision
    EXPECT_LE(
        minimum_distance, 0.01
    ) << "Did not find a minimum distance less than a threshold indicating a collision.";
  }
}
