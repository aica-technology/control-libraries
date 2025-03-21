#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/space/joint/JointTorques.hpp"
#include <gtest/gtest.h>

using namespace state_representation;

static void expect_only_torques(JointTorques& tor) {
  EXPECT_EQ(static_cast<JointState&>(tor).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(tor).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(tor).get_velocities().norm(), 0);
}

TEST(JointTorquesTest, Constructors) {
  JointTorques empty;
  EXPECT_EQ(empty.get_type(), StateType::JOINT_TORQUES);

  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  Eigen::Vector2d torques = Eigen::Vector2d::Random();
  JointTorques jt1("test", torques);
  EXPECT_EQ(jt1.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(jt1.get_name(), "test");
  EXPECT_FALSE(jt1.is_empty());
  EXPECT_EQ(jt1.get_size(), torques.size());
  for (auto i = 0; i < torques.size(); ++i) {
    EXPECT_EQ(jt1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(jt1.data(), torques);

  JointTorques jt2("test", joint_names, torques);
  EXPECT_EQ(jt2.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(jt2.get_name(), "test");
  EXPECT_FALSE(jt2.is_empty());
  EXPECT_EQ(jt2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(jt2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(jt2.data(), torques);
}

TEST(JointTorquesTest, StateCopyConstructor) {
  JointState random_state = JointState::Random("test", 3);
  JointTorques copy1(random_state);
  EXPECT_EQ(copy1.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(random_state.get_name(), copy1.get_name());
  EXPECT_EQ(random_state.get_names(), copy1.get_names());
  EXPECT_EQ(random_state.get_size(), copy1.get_size());
  EXPECT_EQ(random_state.get_torques(), copy1.data());
  expect_only_torques(copy1);

  JointTorques copy2 = random_state;
  EXPECT_EQ(copy2.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(random_state.get_name(), copy2.get_name());
  EXPECT_EQ(random_state.get_names(), copy2.get_names());
  EXPECT_EQ(random_state.get_size(), copy2.get_size());
  EXPECT_EQ(random_state.get_torques(), copy2.data());
  expect_only_torques(copy2);

  JointState empty_state;
  JointTorques copy3(empty_state);
  EXPECT_EQ(copy3.get_type(), StateType::JOINT_TORQUES);
  EXPECT_TRUE(copy3.is_empty());
  JointTorques copy4 = empty_state;
  EXPECT_EQ(copy4.get_type(), StateType::JOINT_TORQUES);
  EXPECT_TRUE(copy4.is_empty());
  JointTorques copy5 = empty_state.copy();
  EXPECT_EQ(copy5.get_type(), StateType::JOINT_TORQUES);
  EXPECT_TRUE(copy5.is_empty());
}

TEST(JointTorquesTest, RandomInitialization) {
  JointTorques random1 = JointTorques::Random("test", 3);
  EXPECT_EQ(random1.get_type(), StateType::JOINT_TORQUES);
  EXPECT_NE(random1.get_torques().norm(), 0);
  expect_only_torques(random1);

  JointTorques random2 = JointTorques::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_EQ(random2.get_type(), StateType::JOINT_TORQUES);
  EXPECT_NE(random2.get_torques().norm(), 0);
  expect_only_torques(random2);
}

TEST(JointTorquesTest, Clamping) {
  JointTorques jt1("test", 3);
  jt1.set_data(-10 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(15);
  EXPECT_EQ(jt1.data(), -10 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(9);
  EXPECT_EQ(jt1.data(), -9 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(20, 0.5);
  EXPECT_EQ(jt1.data(), Eigen::VectorXd::Zero(jt1.get_size()));

  JointTorques jt2("test", 3);
  Eigen::VectorXd torques(3), result(3);
  torques << -2.0, 1.0, -4.0;
  result << -2.0, 0.0, -3.0;
  jt2.set_data(torques);
  jt2.clamp(10);
  EXPECT_EQ(jt2.data(), torques);
  jt2.clamp(3 * Eigen::ArrayXd::Ones(jt2.get_size()), 0.5 * Eigen::ArrayXd::Ones(jt2.get_size()));
  EXPECT_EQ(jt2.data(), result);
}

TEST(JointTorquesTest, GetSetData) {
  JointTorques jt1 = JointTorques::Zero("test", 3);
  JointTorques jt2 = JointTorques::Random("test", 3);
  Eigen::VectorXd data(jt1.get_size());
  data << jt1.get_torques();
  EXPECT_EQ(data, jt1.data());
  for (std::size_t i = 0; i < jt1.get_size(); ++i) {
    EXPECT_EQ(data.array()(i), jt1.array()(i));
  }

  jt1.set_data(jt2.data());
  EXPECT_TRUE(jt2.data().isApprox(jt1.data()));

  auto state_vec = jt2.to_std_vector();
  jt1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), jt1.data()(i));
  }
  EXPECT_THROW(jt1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(JointTorquesTest, ScalarMultiplication) {
  JointTorques jt = JointTorques::Random("test", 3);
  JointTorques jscaled = 0.5 * jt;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(jscaled.data(), 0.5 * jt.data());

  JointTorques empty;
  EXPECT_THROW(0.5 * empty, exceptions::EmptyStateException);
}

TEST(JointTorquesTest, MatrixMultiplication) {
  JointTorques jt = JointTorques::Random("test", 3);
  Eigen::MatrixXd gains = Eigen::VectorXd::Random(jt.get_size()).asDiagonal();

  JointTorques jscaled = gains * jt;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_TORQUES);
  EXPECT_EQ(jscaled.data(), gains * jt.data());

  gains = Eigen::VectorXd::Random(2 * jt.get_size()).asDiagonal();
  EXPECT_THROW(gains * jt, exceptions::IncompatibleSizeException);
}
