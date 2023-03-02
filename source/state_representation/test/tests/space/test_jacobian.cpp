#include "state_representation/space/Jacobian.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include <gtest/gtest.h>

using namespace state_representation;
using namespace state_representation::exceptions;

TEST(JacobianTest, TestCreate) {
  Jacobian jac("robot", 7, "test");
  EXPECT_EQ(jac.get_type(), StateType::JACOBIAN);
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), 7);
  EXPECT_TRUE(jac.is_empty());
  EXPECT_EQ(jac.get_frame(), "test");
  EXPECT_EQ(jac.get_reference_frame(), "world");
  for (std::size_t i = 0; i < jac.cols(); ++i) {
    EXPECT_EQ(jac.get_joint_names().at(i), ("joint" + std::to_string(i)));
    EXPECT_THROW(jac.col(i), EmptyStateException);
  }
  EXPECT_THROW(jac.set_joint_names(5), IncompatibleSizeException);
  EXPECT_THROW(jac.set_joint_names(std::vector<std::string>{"j0"}), IncompatibleSizeException);
  EXPECT_THROW(jac.data(), EmptyStateException);
}

TEST(JacobianTest, TestCreateWithVectorOfJoints) {
  Jacobian jac("robot", std::vector<std::string>{"j1", "j2"}, "test", "test_ref");
  EXPECT_EQ(jac.get_type(), StateType::JACOBIAN);
  EXPECT_EQ(jac.get_joint_names().at(0), "j1");
  EXPECT_EQ(jac.get_joint_names().at(1), "j2");
  EXPECT_EQ(jac.get_reference_frame(), "test_ref");
}

TEST(JacobianTest, TestSetData) {
  Jacobian jac("robot", 3, "test");
  jac.set_data(Eigen::MatrixXd::Random(6, 3));
  EXPECT_FALSE(jac.is_empty());
  for (std::size_t i = 0; i < jac.cols(); ++i) {
    EXPECT_GT(jac.col(i).norm(), 0);
  }
  EXPECT_THROW(jac.set_data(Eigen::MatrixXd::Random(7, 6)), IncompatibleSizeException);
}

TEST(JacobianTest, TestRandomCreate) {
  Jacobian jac = Jacobian::Random("robot", 7, "test");
  EXPECT_EQ(jac.get_type(), StateType::JACOBIAN);
  EXPECT_FALSE(jac.is_empty());
  for (std::size_t i = 0; i < jac.cols(); ++i) {
    EXPECT_GT(jac.col(i).norm(), 0);
  }
}

TEST(JacobianTest, TestTranspose) {
  Jacobian jac = Jacobian::Random("robot", 7, "test");
  auto jact = jac.transpose();

  EXPECT_EQ(jact.rows(), 7);
  EXPECT_EQ(jact.cols(), 6);

  for (std::size_t i = 0; i < jac.cols(); ++i) {
    EXPECT_TRUE(jac.col(i).isApprox(jact.row(i).transpose()));
  }

  jac.reset();
  EXPECT_THROW(jac.transpose(), EmptyStateException);
}

TEST(JacobianTest, TestMutltiplyWithEigen) {
  Jacobian jac = Jacobian::Random("robot", 7, "test");
  Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(7, 2);
  Eigen::MatrixXd res1 = jac * mat1;
  Eigen::MatrixXd res_truth = jac.data() * mat1;

  EXPECT_TRUE(res1.isApprox(res_truth));

  Eigen::MatrixXd mat2 = Eigen::VectorXd::Random(6, 1);
  EXPECT_THROW(jac * mat2, IncompatibleSizeException);

  jac.reset();
  EXPECT_THROW(jac * mat1, EmptyStateException);
}

TEST(JacobianTest, TestInverse) {
  Jacobian jac = Jacobian::Random("robot", 7, "test");
  Eigen::MatrixXd mat1 = Eigen::VectorXd::Random(7, 1);
  EXPECT_THROW(jac.inverse(mat1), IncompatibleSizeException);
  EXPECT_THROW(jac.pseudoinverse(mat1), IncompatibleSizeException);

  Eigen::MatrixXd mat2 = Eigen::VectorXd::Random(6, 1);
  Eigen::MatrixXd res2 = jac.inverse(mat2);
  EXPECT_EQ(res2.rows(), 7);
  EXPECT_EQ(res2.cols(), 1);

  res2 = jac.pseudoinverse(mat2);
  EXPECT_EQ(res2.rows(), 7);
  EXPECT_EQ(res2.cols(), 1);

  jac.reset();
  EXPECT_THROW(jac.inverse(mat2), EmptyStateException);
  EXPECT_THROW(jac.pseudoinverse(mat2), EmptyStateException);
}

TEST(JacobianTest, TestJointToCartesian) {
  Jacobian jac = Jacobian::Random("robot", 7, "test", "test_ref");
  JointVelocities jvel = JointVelocities::Random("robot", 7);
  CartesianTwist cvel = jac * jvel;

  EXPECT_EQ(cvel.get_name(), jac.get_frame());
  EXPECT_EQ(cvel.get_reference_frame(), jac.get_reference_frame());
  EXPECT_TRUE(cvel.data().isApprox(jac.data() * jvel.data()));

  jac.reset();
  EXPECT_THROW(jac * jvel, EmptyStateException);
}

TEST(JacobianTest, TestCartesianToJoint) {
  Jacobian jac = Jacobian::Random("robot", 7, "test", "test_ref");
  CartesianTwist cvel = CartesianTwist::Random("test");
  EXPECT_THROW(jac.pseudoinverse(cvel), IncompatibleStatesException);

  EXPECT_THROW(JointVelocities jvel = jac.inverse(cvel), IncompatibleStatesException);
  EXPECT_THROW(JointVelocities jvel = jac.pseudoinverse(cvel), IncompatibleStatesException);

  cvel.set_reference_frame("test_ref");

  state_representation::JointVelocities jvel2;
  EXPECT_NO_THROW(jvel2 = jac.pseudoinverse(cvel));
  EXPECT_GT(jvel2.data().norm(), 0);
}

TEST(JacobianTest, TestChangeReferenceFrame) {
  Jacobian jac_in_test_ref = Jacobian::Random("robot", 7, "test", "test_ref");
  CartesianPose wTtest_ref = CartesianPose::Random("test_ref", "world");
  Jacobian jac_in_world = wTtest_ref * jac_in_test_ref;
  EXPECT_EQ(jac_in_world.get_reference_frame(), wTtest_ref.get_reference_frame());
  // use a proxy operation with a twist to check correctness
  CartesianTwist vel_in_world = CartesianTwist::Random("test", "world");
  CartesianTwist vel_in_test_ref = wTtest_ref.inverse() * vel_in_world;
  JointVelocities jt1 = jac_in_world.inverse(vel_in_world);
  JointVelocities jt2 = jac_in_test_ref.inverse(vel_in_test_ref);
  EXPECT_TRUE(jt1.data().isApprox(jt2.data()));

  jac_in_test_ref.reset();
  EXPECT_THROW(wTtest_ref * jac_in_test_ref, EmptyStateException);
}

TEST(JacobianTest, Truthiness) {
  Jacobian empty("test", 3, "ee");
  EXPECT_TRUE(empty.is_empty());
  EXPECT_FALSE(empty);

  empty.set_data(Eigen::MatrixXd::Random(6, 3));
  EXPECT_FALSE(empty.is_empty());
  EXPECT_TRUE(empty);
}
