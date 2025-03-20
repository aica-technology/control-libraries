#include <functional>
#include <gtest/gtest.h>

#include "state_representation/MathTools.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/InvalidStateVariableException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"
#include "state_representation/space/cartesian/CartesianAcceleration.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

using namespace state_representation;

static void assert_name_empty_frame_equal(
    const CartesianState& state1, const std::string& name, bool empty, const std::string& reference_frame
) {
  EXPECT_EQ(state1.get_name(), name);
  EXPECT_EQ(state1.get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(state1.is_empty(), empty);
  EXPECT_EQ(state1.get_reference_frame(), reference_frame);
}

static void assert_name_frame_data_equal(const CartesianState& state1, const CartesianState& state2) {
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_EQ(state1.get_reference_frame(), state2.get_reference_frame());
  EXPECT_TRUE(state1.data().isApprox(state2.data()));
}

template<int dim>
void test_clamping(
    CartesianState& state, std::function<Eigen::Matrix<double, dim, 1>(const CartesianState&)> getter,
    std::function<void(CartesianState&, Eigen::Matrix<double, dim, 1>)> setter, const CartesianStateVariable& type
) {
  Eigen::Matrix<double, dim, 1> data;
  if (dim == 3) {
    data << -2.0, 1, 5;
  } else if (dim == 6) {
    data << -2.0, 1, 5, 1, -3.0, 2.4;
  }
  setter(state, data);
  state.clamp_state_variable(10.0, type);
  EXPECT_TRUE(getter(state).isApprox(data));
  state.clamp_state_variable(3.0, type);
  EXPECT_FLOAT_EQ(getter(state).norm(), 3.0);
  state.clamp_state_variable(10.0, type, 0.5);
  EXPECT_FLOAT_EQ(getter(state).norm(), 0.0);
}

TEST(CartesianStateTest, Constructors) {
  CartesianState empty1;
  EXPECT_EQ(empty1.get_type(), StateType::CARTESIAN_STATE);
  assert_name_empty_frame_equal(empty1, "", true, "world");
  EXPECT_THROW(empty1.data(), exceptions::EmptyStateException);

  CartesianState empty2("test");
  assert_name_empty_frame_equal(empty2, "test", true, "world");
  EXPECT_THROW(empty1.data(), exceptions::EmptyStateException);

  CartesianState empty3("test", "reference");
  assert_name_empty_frame_equal(empty3, "test", true, "reference");
  EXPECT_THROW(empty1.data(), exceptions::EmptyStateException);
}

TEST(CartesianStateTest, IdentityInitialization) {
  CartesianState identity = CartesianState::Identity("test");
  EXPECT_FALSE(identity.is_empty());
  EXPECT_FLOAT_EQ(identity.get_position().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_orientation().norm(), 1);
  EXPECT_FLOAT_EQ(identity.get_orientation().w(), 1);
  EXPECT_FLOAT_EQ(identity.get_twist().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_acceleration().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, RandomStateInitialization) {
  CartesianState random = CartesianState::Random("test");
  EXPECT_NE(random.get_position().norm(), 0);
  EXPECT_FLOAT_EQ(random.get_orientation().norm(), 1);
  EXPECT_NE(random.get_orientation().w(), 0);
  EXPECT_NE(random.get_orientation().x(), 0);
  EXPECT_NE(random.get_orientation().y(), 0);
  EXPECT_NE(random.get_orientation().z(), 0);
  EXPECT_NE(random.get_twist().norm(), 0);
  EXPECT_NE(random.get_acceleration().norm(), 0);
  EXPECT_NE(random.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, CopyConstructor) {
  CartesianState random = CartesianState::Random("test");
  CartesianState copy1(random);
  assert_name_frame_data_equal(random, copy1);

  CartesianState copy2 = random;
  assert_name_frame_data_equal(random, copy2);

  CartesianState copy3 = random.copy();
  assert_name_frame_data_equal(random, copy3);

  CartesianState empty;
  CartesianState copy4(empty);
  EXPECT_TRUE(copy4.is_empty());
  CartesianState copy5 = empty;
  EXPECT_TRUE(copy5.is_empty());
  CartesianState copy6 = empty.copy();
  EXPECT_TRUE(copy6.is_empty());
}

TEST(CartesianStateTest, GetSetFields) {
  CartesianState cs("test");
  static Eigen::Vector3d data;
  static std::vector<double> std_data(3);

  // name
  cs.set_name("robot");
  EXPECT_EQ(cs.get_name(), "robot");
  EXPECT_EQ(cs.get_reference_frame(), "world");
  cs.set_reference_frame("base");
  EXPECT_EQ(cs.get_reference_frame(), "base");

  // position
  std_data = {1, 2, 3};
  cs.set_position(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_position()(i), std_data.at(i));
  }
  cs.set_position(1.1, 2.2, 3.3);
  EXPECT_TRUE(Eigen::Vector3d(1.1, 2.2, 3.3).isApprox(cs.get_position()));
  EXPECT_THROW(cs.set_position(std::vector<double>{1, 2, 3, 4}), exceptions::IncompatibleSizeException);

  // orientation
  Eigen::Vector4d orientation_vec = Eigen::Vector4d::Random().normalized();
  cs.set_orientation(orientation_vec);
  for (auto i = 0; i < orientation_vec.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_orientation_coefficients()(i), orientation_vec(i));
  }
  Eigen::Quaterniond random_orientation = Eigen::Quaterniond::UnitRandom();
  std::vector<double> orientation{
      random_orientation.w(), random_orientation.x(), random_orientation.y(), random_orientation.z()
  };
  cs.set_orientation(orientation);
  EXPECT_TRUE(random_orientation.coeffs().isApprox(cs.get_orientation().coeffs()));
  random_orientation = Eigen::Quaterniond::UnitRandom();
  cs.set_orientation(random_orientation.w(), random_orientation.x(), random_orientation.y(), random_orientation.z());
  EXPECT_TRUE(random_orientation.coeffs().isApprox(cs.get_orientation().coeffs()));
  EXPECT_THROW(cs.set_orientation(std_data), exceptions::IncompatibleSizeException);

  auto matrix = cs.get_transformation_matrix();
  Eigen::Vector3d trans = matrix.topRightCorner<3, 1>();
  Eigen::Matrix3d rot = matrix.topLeftCorner<3, 3>();
  Eigen::Vector4d bottom = matrix.bottomLeftCorner<1, 4>();
  EXPECT_TRUE(trans.isApprox(cs.get_position()));
  EXPECT_TRUE(rot.isApprox(random_orientation.toRotationMatrix()));
  EXPECT_TRUE(bottom.isApprox(Eigen::Vector4d(0, 0, 0, 1)));

  // pose
  EXPECT_THROW(cs.set_pose(std::vector<double>(8)), exceptions::IncompatibleSizeException);
  auto cs2 = CartesianState::Random(cs.get_name());
  EXPECT_FALSE(cs2.get_pose().isApprox(cs.get_pose()));
  cs2.set_pose_from_transformation_matrix(cs.get_transformation_matrix());
  EXPECT_TRUE(cs2.get_position().isApprox(cs.get_position()));
  EXPECT_TRUE(cs2.get_orientation().angularDistance(cs.get_orientation()) < 1e-3);

  // linear velocity
  data = Eigen::Vector3d::Random();
  cs.set_linear_velocity(data);
  EXPECT_TRUE(cs.get_linear_velocity().isApprox(data));
  std_data = {2, 3, 4};
  cs.set_linear_velocity(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_linear_velocity()(i), std_data.at(i));
  }
  cs.set_linear_velocity(2.1, 3.2, 4.3);
  EXPECT_TRUE(Eigen::Vector3d(2.1, 3.2, 4.3).isApprox(cs.get_linear_velocity()));
  EXPECT_THROW(cs.set_linear_velocity(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // angular velocity
  data = Eigen::Vector3d::Random();
  cs.set_angular_velocity(data);
  EXPECT_TRUE(cs.get_angular_velocity().isApprox(data));
  std_data = {3, 4, 5};
  cs.set_angular_velocity(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_angular_velocity()(i), std_data.at(i));
  }
  cs.set_angular_velocity(3.1, 4.2, 5.3);
  EXPECT_TRUE(Eigen::Vector3d(3.1, 4.2, 5.3).isApprox(cs.get_angular_velocity()));
  EXPECT_THROW(cs.set_angular_velocity(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // twist
  Eigen::VectorXd twist_eigen = Eigen::VectorXd::Random(6);
  cs.set_twist(twist_eigen);
  EXPECT_TRUE(cs.get_twist().isApprox(twist_eigen));
  std::vector<double> twist{4, 5, 6, 7, 8, 9};
  cs.set_twist(twist);
  for (std::size_t i = 0; i < twist.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_twist()(i), twist.at(i));
  }
  EXPECT_THROW(cs.set_twist(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // linear acceleration
  data = Eigen::Vector3d::Random();
  cs.set_linear_acceleration(data);
  EXPECT_TRUE(cs.get_linear_acceleration().isApprox(data));
  std_data = {5, 6, 7};
  cs.set_linear_acceleration(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_linear_acceleration()(i), std_data.at(i));
  }
  cs.set_linear_acceleration(5.1, 6.2, 7.3);
  EXPECT_TRUE(Eigen::Vector3d(5.1, 6.2, 7.3).isApprox(cs.get_linear_acceleration()));
  EXPECT_THROW(cs.set_linear_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // angular acceleration
  data = Eigen::Vector3d::Random();
  cs.set_angular_acceleration(data);
  EXPECT_TRUE(cs.get_angular_acceleration().isApprox(data));
  std_data = {6, 7, 8};
  cs.set_angular_acceleration(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_angular_acceleration()(i), std_data.at(i));
  }
  cs.set_angular_acceleration(6.1, 7.2, 8.3);
  EXPECT_TRUE(Eigen::Vector3d(6.1, 7.2, 8.3).isApprox(cs.get_angular_acceleration()));
  EXPECT_THROW(cs.set_angular_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // acceleration
  Eigen::VectorXd acceleration_eigen = Eigen::VectorXd::Random(6);
  cs.set_acceleration(acceleration_eigen);
  EXPECT_TRUE(cs.get_acceleration().isApprox(acceleration_eigen));
  std::vector<double> acceleration{7, 8, 9, 10, 11, 12};
  cs.set_acceleration(acceleration);
  for (std::size_t i = 0; i < acceleration.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_acceleration()(i), acceleration.at(i));
  }
  EXPECT_THROW(cs.set_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // force
  data = Eigen::Vector3d::Random();
  cs.set_force(data);
  EXPECT_TRUE(cs.get_force().isApprox(data));
  std_data = {8, 9, 10};
  cs.set_force(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_force()(i), std_data.at(i));
  }
  cs.set_force(8.1, 9.2, 10.3);
  EXPECT_TRUE(Eigen::Vector3d(8.1, 9.2, 10.3).isApprox(cs.get_force()));
  EXPECT_THROW(cs.set_force(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // torque
  data = Eigen::Vector3d::Random();
  cs.set_torque(data);
  EXPECT_TRUE(cs.get_torque().isApprox(data));
  std_data = {9, 10, 11};
  cs.set_torque(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_torque()(i), std_data.at(i));
  }
  cs.set_torque(9.1, 10.2, 11.3);
  EXPECT_TRUE(Eigen::Vector3d(9.1, 10.2, 11.3).isApprox(cs.get_torque()));
  EXPECT_THROW(cs.set_torque(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // wrench
  Eigen::VectorXd wrench_eigen = Eigen::VectorXd::Random(6);
  cs.set_wrench(wrench_eigen);
  EXPECT_TRUE(cs.get_wrench().isApprox(wrench_eigen));
  std::vector<double> wrench{10, 11, 12, 13, 14, 15};
  cs.set_wrench(wrench);
  for (std::size_t i = 0; i < wrench.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_wrench()(i), wrench.at(i));
  }
  EXPECT_THROW(cs.set_wrench(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  cs.set_zero();
  EXPECT_FLOAT_EQ(cs.data().norm(), 1);
  EXPECT_EQ(cs.is_empty(), false);
  cs.reset();
  EXPECT_THROW(cs.data(), exceptions::EmptyStateException);
  EXPECT_EQ(cs.is_empty(), true);
}

TEST(CartesianStateTest, SetZero) {
  CartesianState random1 = CartesianState::Random("test");
  EXPECT_FALSE(random1.is_empty());
  random1.reset();
  EXPECT_TRUE(random1.is_empty());
  EXPECT_THROW(random1.data(), exceptions::EmptyStateException);

  CartesianState random2 = CartesianState::Random("test");
  random2.set_zero();
  EXPECT_FLOAT_EQ(random2.data().norm(), 1);
}

TEST(CartesianStateTest, GetSetData) {
  CartesianState cs1 = CartesianState::Identity("test");
  CartesianState cs2 = CartesianState::Random("test");
  Eigen::VectorXd concatenated_state(25);
  concatenated_state << cs1.get_pose(), cs1.get_twist(), cs1.get_acceleration(), cs1.get_wrench();
  EXPECT_TRUE(concatenated_state.isApprox(cs1.data()));
  for (std::size_t i = 0; i < 25; ++i) {
    EXPECT_FLOAT_EQ(concatenated_state.array()(i), cs1.array()(i));
  }

  cs1.set_data(cs2.data());
  EXPECT_TRUE(cs1.data().isApprox(cs2.data()));

  cs2 = CartesianState::Random("test");
  auto state_vec = cs2.to_std_vector();
  cs1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_FLOAT_EQ(state_vec.at(i), cs1.data()(i));
  }
  EXPECT_THROW(cs1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(CartesianStateTest, ClampVariable) {
  CartesianState state = CartesianState::Identity("test");
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ORIENTATION), exceptions::NotImplementedException);
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::POSE), exceptions::NotImplementedException);
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ALL), exceptions::NotImplementedException);

  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_position(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_position(data); },
      CartesianStateVariable::POSITION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_linear_velocity(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_linear_velocity(data); },
      CartesianStateVariable::LINEAR_VELOCITY
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_angular_velocity(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_angular_velocity(data); },
      CartesianStateVariable::ANGULAR_VELOCITY
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::Matrix<double, 6, 1> { return state.get_twist(); },
      [](CartesianState& state, const Eigen::Matrix<double, 6, 1>& data) { state.set_twist(data); },
      CartesianStateVariable::TWIST
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_linear_acceleration(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_linear_acceleration(data); },
      CartesianStateVariable::LINEAR_ACCELERATION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_angular_acceleration(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_angular_acceleration(data); },
      CartesianStateVariable::ANGULAR_ACCELERATION
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::MatrixXd { return state.get_acceleration(); },
      [](CartesianState& state, const Eigen::MatrixXd& data) { state.set_acceleration(data); },
      CartesianStateVariable::ACCELERATION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_force(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_force(data); }, CartesianStateVariable::FORCE
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_torque(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_torque(data); }, CartesianStateVariable::TORQUE
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::MatrixXd { return state.get_wrench(); },
      [](CartesianState& state, const Eigen::MatrixXd& data) { state.set_wrench(data); }, CartesianStateVariable::WRENCH
  );
}

TEST(CartesianStateTest, Norms) {
  CartesianState cs = CartesianState::Random("test");
  std::vector<double> norms = cs.norms();
  ASSERT_TRUE(norms.size() == 8);
  EXPECT_FLOAT_EQ(norms[0], cs.get_position().norm());
  EXPECT_FLOAT_EQ(norms[1], cs.get_orientation().norm());
  EXPECT_FLOAT_EQ(norms[2], cs.get_linear_velocity().norm());
  EXPECT_FLOAT_EQ(norms[3], cs.get_angular_velocity().norm());
  EXPECT_FLOAT_EQ(norms[4], cs.get_linear_acceleration().norm());
  EXPECT_FLOAT_EQ(norms[5], cs.get_angular_acceleration().norm());
  EXPECT_FLOAT_EQ(norms[6], cs.get_force().norm());
  EXPECT_FLOAT_EQ(norms[7], cs.get_torque().norm());
}

TEST(CartesianStateTest, Normalize) {
  CartesianState cs = CartesianState::Random("test");
  auto normalized = cs.normalized();
  std::vector<double> norms1 = normalized.norms();
  for (double n : norms1) {
    EXPECT_FLOAT_EQ(n, 1.0);
  }
  cs.normalize();
  std::vector<double> norms2 = cs.norms();
  for (double n : norms2) {
    EXPECT_FLOAT_EQ(n, 1.0);
  }
}

TEST(CartesianStateTest, Distance) {
  CartesianState empty;
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test", "robot");
  EXPECT_THROW(empty.dist(cs1), exceptions::EmptyStateException);
  EXPECT_THROW(cs1.dist(empty), exceptions::EmptyStateException);
  EXPECT_THROW(cs1.dist(cs2), exceptions::IncompatibleReferenceFramesException);

  Eigen::VectorXd data1 = Eigen::VectorXd::Random(25);
  cs1.set_data(data1);
  CartesianState cs3 = CartesianState("test");
  Eigen::VectorXd data3 = Eigen::VectorXd::Random(25);
  cs3.set_data(data3);

  double pos_dist = (data1.head(3) - data3.head(3)).norm();
  double inner_product = cs1.get_orientation().dot(cs3.get_orientation());
  double orient_dist = acos(std::min(1.0, std::max(-1.0, 2 * inner_product * inner_product - 1)));
  double lin_vel_dist = (data1.segment(7, 3) - data3.segment(7, 3)).norm();
  double ang_vel_dist = (data1.segment(10, 3) - data3.segment(10, 3)).norm();
  double lin_acc_dist = (data1.segment(13, 3) - data3.segment(13, 3)).norm();
  double ang_acc_dist = (data1.segment(16, 3) - data3.segment(16, 3)).norm();
  double force_dist = (data1.segment(19, 3) - data3.segment(19, 3)).norm();
  double torque_dist = (data1.segment(22, 3) - data3.segment(22, 3)).norm();
  double total_dist =
      pos_dist + orient_dist + lin_vel_dist + ang_vel_dist + lin_acc_dist + ang_acc_dist + force_dist + torque_dist;

  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::POSITION), pos_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ORIENTATION), orient_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::POSE), pos_dist + orient_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::TWIST), lin_vel_dist + ang_vel_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ACCELERATION), lin_acc_dist + ang_acc_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::WRENCH), force_dist + torque_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ALL), total_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3), cs3.dist(cs1));
}

TEST(CartesianStateTest, Inverse) {
  auto a_state_b = CartesianState::Random("B", "A");
  auto b_state_a = a_state_b.inverse();

  // frame of reference should be flipped
  EXPECT_STREQ(b_state_a.get_name().c_str(), a_state_b.get_reference_frame().c_str());
  EXPECT_STREQ(b_state_a.get_reference_frame().c_str(), a_state_b.get_name().c_str());

  // the wrench should be set to zero
  EXPECT_EQ(b_state_a.get_wrench().sum(), 0);

  // the double inverse should be the same as the original state, excluding the wrench
  a_state_b.set_wrench(Eigen::Vector<double, 6>::Zero());
  auto new_a_state_b = b_state_a.inverse();
  EXPECT_STREQ(new_a_state_b.get_name().c_str(), a_state_b.get_name().c_str());
  EXPECT_STREQ(new_a_state_b.get_reference_frame().c_str(), a_state_b.get_reference_frame().c_str());
  EXPECT_NEAR(a_state_b.dist(new_a_state_b, CartesianStateVariable::ALL), 0, 1e-5);

  // the product of a state and its inverse should result in an identity / zero state
  // (wrench is the exception because the transform product operation is handled differently)
  auto expect_null = a_state_b * b_state_a;
  EXPECT_STREQ(expect_null.get_name().c_str(), expect_null.get_reference_frame().c_str());
  EXPECT_STREQ(expect_null.get_name().c_str(), a_state_b.get_reference_frame().c_str());
  EXPECT_NEAR(expect_null.get_pose().norm(), 1, 1e-5);
  EXPECT_NEAR(expect_null.get_linear_velocity().norm(), 0, 1e-5);
  EXPECT_NEAR(expect_null.get_angular_velocity().norm(), 0, 1e-5);
  EXPECT_NEAR(expect_null.get_linear_acceleration().norm(), 0, 1e-5);
  EXPECT_NEAR(expect_null.get_angular_acceleration().norm(), 0, 1e-5);
}

TEST(CartesianStateTest, InverseStaticFrame) {
  auto state = CartesianState::Identity("test");
  auto random = CartesianState::Random("random");

  // with null orientation, the inverse position is simply negated
  state.set_position(random.get_position());
  auto inverse = state.inverse();
  EXPECT_EQ(inverse.get_position().x(), -state.get_position().x());
  EXPECT_EQ(inverse.get_position().y(), -state.get_position().y());
  EXPECT_EQ(inverse.get_position().z(), -state.get_position().z());

  // with some orientation, the inverse position is negated and rotated
  // apply a 90-degree rotation around X
  state.set_orientation(0.707, 0.707, 0.0, 0.0);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_position().x(), -state.get_position().x());
  EXPECT_FLOAT_EQ(inverse.get_position().y(), -state.get_position().z());
  EXPECT_FLOAT_EQ(inverse.get_position().z(), state.get_position().y());
  EXPECT_NEAR((state * state.inverse()).get_pose().norm(), 1, 1e-5);

  // apply a 90-degree rotation around Y
  state.set_orientation(0.707, 0.0, 0.707, 0.0);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_position().x(), state.get_position().z());
  EXPECT_FLOAT_EQ(inverse.get_position().y(), -state.get_position().y());
  EXPECT_FLOAT_EQ(inverse.get_position().z(), -state.get_position().x());

  // apply a 90-degree rotation around Z
  state.set_orientation(0.707, 0.0, 0.0, 0.707);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_position().x(), -state.get_position().y());
  EXPECT_FLOAT_EQ(inverse.get_position().y(), state.get_position().x());
  EXPECT_FLOAT_EQ(inverse.get_position().z(), -state.get_position().z());

  // for any pose, the inverse orientation is simply the conjugate
  state.set_pose(random.get_pose());
  inverse = state.inverse();
  Eigen::Quaterniond q = state.get_orientation().conjugate();
  EXPECT_FLOAT_EQ(inverse.get_orientation().w(), q.w());
  EXPECT_FLOAT_EQ(inverse.get_orientation().x(), q.x());
  EXPECT_FLOAT_EQ(inverse.get_orientation().y(), q.y());
  EXPECT_FLOAT_EQ(inverse.get_orientation().z(), q.z());

  // for any pose, the inverse position is negated and rotated by the conjugate orientation
  Eigen::Vector3d p = q * state.get_position();
  EXPECT_FLOAT_EQ(inverse.get_position().x(), -p.x());
  EXPECT_FLOAT_EQ(inverse.get_position().y(), -p.y());
  EXPECT_FLOAT_EQ(inverse.get_position().z(), -p.z());
}

TEST(CartesianStateTest, InverseMovingFrame) {
  auto state = CartesianState::Identity("test");
  auto random = CartesianState::Random("random");

  // with no pose offset, the twist is simply negated
  state.set_twist(random.get_twist());
  auto inverse = state.inverse();
  EXPECT_EQ(inverse.get_linear_velocity().x(), -state.get_linear_velocity().x());
  EXPECT_EQ(inverse.get_linear_velocity().y(), -state.get_linear_velocity().y());
  EXPECT_EQ(inverse.get_linear_velocity().z(), -state.get_linear_velocity().z());
  EXPECT_EQ(inverse.get_angular_velocity().x(), -state.get_angular_velocity().x());
  EXPECT_EQ(inverse.get_angular_velocity().y(), -state.get_angular_velocity().y());
  EXPECT_EQ(inverse.get_angular_velocity().z(), -state.get_angular_velocity().z());

  // with no orientation offset, the inverse angular velocity is always the negative for any position or linear velocity
  state.set_position(random.get_position());
  inverse = state.inverse();
  EXPECT_EQ(inverse.get_angular_velocity().x(), -state.get_angular_velocity().x());
  EXPECT_EQ(inverse.get_angular_velocity().y(), -state.get_angular_velocity().y());
  EXPECT_EQ(inverse.get_angular_velocity().z(), -state.get_angular_velocity().z());

  // with no orientation offset, the inverse linear velocity is affected by the position offset and angular velocity
  // according to the cross product of the pose and angular velocity vectors
  state.set_position(1, 0, 0);
  inverse = state.inverse();
  EXPECT_EQ(inverse.get_linear_velocity().x(), -state.get_linear_velocity().x());
  EXPECT_EQ(
      inverse.get_linear_velocity().y(),
      -state.get_linear_velocity().y() + state.get_angular_velocity().z() * state.get_position().x()
  );
  EXPECT_EQ(
      inverse.get_linear_velocity().z(),
      -state.get_linear_velocity().z() - state.get_angular_velocity().y() * state.get_position().x()
  );

  state.set_position(0, 1, 0);
  inverse = state.inverse();
  EXPECT_EQ(
      inverse.get_linear_velocity().x(),
      -state.get_linear_velocity().x() - state.get_angular_velocity().z() * state.get_position().y()
  );
  EXPECT_EQ(inverse.get_linear_velocity().y(), -state.get_linear_velocity().y());
  EXPECT_EQ(
      inverse.get_linear_velocity().z(),
      -state.get_linear_velocity().z() + state.get_angular_velocity().x() * state.get_position().y()
  );

  state.set_position(0, 0, 1);
  inverse = state.inverse();
  EXPECT_EQ(
      inverse.get_linear_velocity().x(),
      -state.get_linear_velocity().x() + state.get_angular_velocity().y() * state.get_position().z()
  );
  EXPECT_EQ(
      inverse.get_linear_velocity().y(),
      -state.get_linear_velocity().y() - state.get_angular_velocity().x() * state.get_position().z()
  );
  EXPECT_EQ(inverse.get_linear_velocity().z(), -state.get_linear_velocity().z());

  // general case with any position and no orientation offset
  state.set_position(random.get_position());
  inverse = state.inverse();
  Eigen::Vector3d v = -state.get_linear_velocity() + state.get_angular_velocity().cross(state.get_position());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().x(), v.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().y(), v.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().z(), v.z());

  // with orientation offset and no position offset, the linear and angular velocities are simply negated and rotated
  state.set_position(0, 0, 0);
  state.set_linear_velocity(0, 0, 0);
  state.set_angular_velocity(10, 20, 30);

  // apply a 90-degree rotation around X
  state.set_orientation(0.707, 0.707, 0.0, 0.0);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().x(), -state.get_angular_velocity().x());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().y(), -state.get_angular_velocity().z());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().z(), state.get_angular_velocity().y());

  // apply a 90-degree rotation around Y
  state.set_orientation(0.707, 0.0, 0.707, 0.0);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().x(), state.get_angular_velocity().z());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().y(), -state.get_angular_velocity().y());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().z(), -state.get_angular_velocity().x());

  // apply a 90-degree rotation around Z
  state.set_orientation(0.707, 0.0, 0.0, 0.707);
  inverse = state.inverse();
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().x(), -state.get_angular_velocity().y());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().y(), state.get_angular_velocity().x());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().z(), -state.get_angular_velocity().z());

  // general case with any orientation and no position offset
  state.set_orientation(random.get_orientation());
  state.set_twist(random.get_twist());
  inverse = state.inverse();
  v = state.get_orientation().conjugate() * (-state.get_linear_velocity());
  Eigen::Vector3d w = state.get_orientation().conjugate() * (-state.get_angular_velocity());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().x(), v.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().y(), v.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().z(), v.z());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().x(), w.x());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().y(), w.y());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().z(), w.z());

  // in the general case for any pose and twist, the inverse angular velocity is simply negated and rotated, while the
  // inverse linear velocity is negated, rotated and offset by the cross product of the angular velocity and pose.
  state.set_pose(random.get_pose());
  state.set_twist(random.get_twist());
  inverse = state.inverse();
  v = state.get_orientation().conjugate() * (-state.get_linear_velocity());
  w = state.get_orientation().conjugate() * (-state.get_angular_velocity());
  v += w.cross(inverse.get_position());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().x(), v.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().y(), v.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_velocity().z(), v.z());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().x(), w.x());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().y(), w.y());
  EXPECT_FLOAT_EQ(inverse.get_angular_velocity().z(), w.z());
}

TEST(CartesianStateTest, InverseAcceleratingFrame) {
  auto state = CartesianState::Identity("test");
  auto random = CartesianState::Random("random");

  // with no initial pose or twist, the inverse acceleration is simply negated
  state.set_acceleration(random.get_acceleration());
  auto inverse = state.inverse();
  EXPECT_EQ(inverse.get_linear_acceleration().x(), -state.get_linear_acceleration().x());
  EXPECT_EQ(inverse.get_linear_acceleration().y(), -state.get_linear_acceleration().y());
  EXPECT_EQ(inverse.get_linear_acceleration().z(), -state.get_linear_acceleration().z());
  EXPECT_EQ(inverse.get_angular_acceleration().x(), -state.get_angular_acceleration().x());
  EXPECT_EQ(inverse.get_angular_acceleration().y(), -state.get_angular_acceleration().y());
  EXPECT_EQ(inverse.get_angular_acceleration().z(), -state.get_angular_acceleration().z());

  // if there is only an orientation offset, the inverse acceleration is negated and rotated
  state.set_orientation(random.get_orientation());
  inverse = state.inverse();
  Eigen::Vector3d av = state.get_orientation().conjugate() * (-state.get_linear_acceleration());
  Eigen::Vector3d aw = state.get_orientation().conjugate() * (-state.get_angular_acceleration());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().x(), av.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().y(), av.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().z(), av.z());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().x(), aw.x());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().y(), aw.y());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().z(), aw.z());

  // for any state, the inverse angular acceleration depends only on the orientation
  state.set_data(random.data());
  inverse = state.inverse();
  aw = state.get_orientation().conjugate() * (-state.get_angular_acceleration());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().x(), aw.x());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().y(), aw.y());
  EXPECT_FLOAT_EQ(inverse.get_angular_acceleration().z(), aw.z());

  // the inverse linear acceleration depends on position, orientation, linear velocity, angular velocity and angular acceleration.
  // for the following tests, the base linear acceleration is set to zero to look only at the additional terms.
  state.set_linear_acceleration(0, 0, 0);

  // if there is position offset and angular acceleration, the additional inverse linear acceleration is the Euler acceleration,
  // the cross product of the inverted angular acceleration and distance
  state.set_orientation(1, 0, 0, 0);
  state.set_linear_velocity(0, 0, 0);
  state.set_angular_velocity(0, 0, 0);
  state.set_angular_acceleration(random.get_angular_acceleration());
  inverse = state.inverse();
  av = inverse.get_angular_acceleration().cross(inverse.get_position());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().x(), av.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().y(), av.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().z(), av.z());

  // if there is only twist and no pose offset, the additional inverse linear acceleration is the Coriolis acceleration,
  // twice the cross product of the inverted angular and linear velocity
  state.set_position(0, 0, 0);
  state.set_orientation(1, 0, 0, 0);
  state.set_twist(random.get_twist());
  state.set_angular_acceleration(0, 0, 0);
  inverse = state.inverse();
  av = 2 * (inverse.get_angular_velocity()).cross(inverse.get_linear_velocity());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().x(), av.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().y(), av.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().z(), av.z());

  // if there is twist and position offset, the additional inverse linear acceleration is the centrifugal acceleration
  state.set_position(random.get_position());
  inverse = state.inverse();
  av = 2 * (inverse.get_angular_velocity()).cross(inverse.get_linear_velocity());// Coriolis
  av +=
      -inverse.get_angular_velocity().cross(inverse.get_angular_velocity().cross(inverse.get_position()));// centrifugal
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().x(), av.x());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().y(), av.y());
  EXPECT_FLOAT_EQ(inverse.get_linear_acceleration().z(), av.z());
}

TEST(CartesianStateTest, Addition) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test");
  CartesianState cs3 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 + cs3, exceptions::IncompatibleReferenceFramesException);

  CartesianState csum = cs1 + cs2;
  EXPECT_TRUE(csum.get_position().isApprox(cs1.get_position() + cs2.get_position()));
  auto orientation = cs1.get_orientation() * cs2.get_orientation();
  if (orientation.dot(cs1.get_orientation()) < 0) {
    orientation = Eigen::Quaterniond(-orientation.coeffs());
  }
  EXPECT_TRUE(csum.get_orientation().coeffs().isApprox(orientation.coeffs()));
  EXPECT_TRUE(csum.get_twist().isApprox(cs1.get_twist() + cs2.get_twist()));
  EXPECT_TRUE(csum.get_acceleration().isApprox(cs1.get_acceleration() + cs2.get_acceleration()));
  EXPECT_TRUE(csum.get_wrench().isApprox(cs1.get_wrench() + cs2.get_wrench()));

  cs1 += cs2;
  EXPECT_TRUE(cs1.data().isApprox(csum.data()));
}

TEST(CartesianStateTest, Subtraction) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 - cs2, exceptions::IncompatibleReferenceFramesException);

  auto cdiff = cs1 - cs1;
  EXPECT_FLOAT_EQ(cdiff.data().norm(), 1.);
  EXPECT_FLOAT_EQ(abs(cdiff.get_orientation().w()), 1.);

  cs1 -= cs1;
  EXPECT_TRUE(cs1.data().isApprox(cdiff.data()));

  auto res = cs1 + cs1 - cs1;
  EXPECT_TRUE(res.data().isApprox(cs1.data()));
}

TEST(CartesianStateTest, ScalarMultiplication) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = scalar * cs;
  EXPECT_TRUE(cscaled.get_position().isApprox(scalar * cs.get_position()));
  auto qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), scalar);
  EXPECT_TRUE(cscaled.get_orientation().angularDistance(qscaled) < 1e-3);
  EXPECT_TRUE(cscaled.get_twist().isApprox(scalar * cs.get_twist()));
  EXPECT_TRUE(cscaled.get_acceleration().isApprox(scalar * cs.get_acceleration()));
  EXPECT_TRUE(cscaled.get_wrench().isApprox(scalar * cs.get_wrench()));
  EXPECT_TRUE((cs * scalar).data().isApprox(cscaled.data()));
  cs *= scalar;
  EXPECT_TRUE(cscaled.data().isApprox(cs.data()));

  CartesianState empty;
  EXPECT_THROW(scalar * empty, exceptions::EmptyStateException);
}

TEST(CartesianStateTest, ScalarDivision) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = cs / scalar;
  EXPECT_TRUE(cscaled.get_position().isApprox(cs.get_position() / scalar));
  auto qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), 1.0 / scalar);
  EXPECT_TRUE(cscaled.get_orientation().angularDistance(qscaled) < 1e-3);
  EXPECT_TRUE(cscaled.get_twist().isApprox(cs.get_twist() / scalar));
  EXPECT_TRUE(cscaled.get_acceleration().isApprox(cs.get_acceleration() / scalar));
  EXPECT_TRUE(cscaled.get_wrench().isApprox(cs.get_wrench() / scalar));
  cs /= scalar;
  EXPECT_TRUE(cscaled.data().isApprox(cs.data()));

  EXPECT_THROW(cs / 0.0, std::runtime_error);

  CartesianState empty;
  EXPECT_THROW(empty / scalar, exceptions::EmptyStateException);
}

TEST(CartesianStateTest, OrientationScaling) {
  auto cs = CartesianState::Random("A");

  for (int i = 0; i < 5; ++i) {
    double scale = static_cast<double>(rand()) / RAND_MAX + i;

    auto qscaled = Eigen::Quaterniond::Identity();
    auto cscaled = scale * cs;
    for (int j = 0; j < i; ++j) {
      qscaled = qscaled * cs.get_orientation();
    }
    qscaled = qscaled * Eigen::Quaterniond::Identity().slerp(scale - i, cs.get_orientation());
    EXPECT_LT(cscaled.get_orientation().angularDistance(qscaled), 1e-3);

    qscaled = Eigen::Quaterniond::Identity();
    cscaled = -scale * cs;
    for (int j = 0; j < i; ++j) {
      qscaled = qscaled * cs.get_orientation();
    }
    qscaled = qscaled * Eigen::Quaterniond::Identity().slerp(scale - i, cs.get_orientation());
    EXPECT_LT(cscaled.get_orientation().angularDistance(qscaled.conjugate()), 1e-3);
  }
}

TEST(CartesianStateTest, Multiplication) {
  CartesianState world_cs_first = CartesianState::Random("first");
  CartesianState first_cs_second = CartesianState::Random("second", "first");
  CartesianState world_cs_second = world_cs_first * first_cs_second;

  EXPECT_STREQ(world_cs_second.get_name().c_str(), first_cs_second.get_name().c_str());
  EXPECT_STREQ(world_cs_second.get_reference_frame().c_str(), world_cs_first.get_reference_frame().c_str());

  // only the wrench of the second state is preserved, but aligned with the base frame
  auto expect_wrench = CartesianPose(world_cs_first) * CartesianWrench(first_cs_second);
  EXPECT_FLOAT_EQ(world_cs_second.dist(expect_wrench, CartesianStateVariable::WRENCH), 0);
}

TEST(CartesianStateTest, Truthiness) {
  CartesianState empty("test");
  EXPECT_TRUE(empty.is_empty());
  EXPECT_FALSE(empty);

  empty.set_data(Eigen::VectorXd::Random(25));
  EXPECT_FALSE(empty.is_empty());
  EXPECT_TRUE(empty);
}

TEST(CartesianStateTest, TestMultiplicationOperators) {
  // to have fully compatible states, use "world" as name and reference frame
  CartesianState state = CartesianState::Random("world");
  CartesianPose pose = CartesianPose::Random("world");
  CartesianTwist twist = CartesianTwist::Random("world");
  CartesianAcceleration acc = CartesianAcceleration::Random("world");
  CartesianWrench wrench = CartesianWrench::Random("world");

  // CartesianState multiplied with any derived stays a CartesianState
  auto r1 = state * state;
  EXPECT_EQ(r1.get_type(), StateType::CARTESIAN_STATE);
  auto r2 = state * pose;
  EXPECT_EQ(r2.get_type(), StateType::CARTESIAN_STATE);
  auto r3 = state * twist;
  EXPECT_EQ(r3.get_type(), StateType::CARTESIAN_STATE);
  auto r4 = state * acc;
  EXPECT_EQ(r4.get_type(), StateType::CARTESIAN_STATE);
  auto r5 = state * wrench;
  EXPECT_EQ(r5.get_type(), StateType::CARTESIAN_STATE);

  // CartesianPose multiplied with any derived type is defined by the right hand type
  auto r6 = pose * state;
  EXPECT_EQ(r6.get_type(), StateType::CARTESIAN_STATE);
  auto r7 = pose * pose;
  EXPECT_EQ(r7.get_type(), StateType::CARTESIAN_POSE);
  auto r8 = pose * twist;
  EXPECT_EQ(r8.get_type(), StateType::CARTESIAN_TWIST);
  auto r9 = pose * acc;
  EXPECT_EQ(r9.get_type(), StateType::CARTESIAN_ACCELERATION);
  auto r10 = pose * wrench;
  EXPECT_EQ(r10.get_type(), StateType::CARTESIAN_WRENCH);

  // COMMENTED TEST BELOW EXPECTED TO BE NOT COMPILABLE

  //auto r11 = twist * state;
  //auto r12 = twist * pose;
  //auto r13 = twist * twist;
  //auto r14 = twist * acc;
  //auto r15 = twist * wrench;
  //auto r21 = acc * state;
  //auto r22 = acc * pose;
  //auto r23 = acc * twist;
  //auto r24 = acc * acc;
  //auto r25 = acc * wrench;
  //auto r16 = wrench * state;
  //auto r17 = wrench * pose;
  //auto r18 = wrench * twist;
  //auto r19 = wrench * acc;
  //auto r20 = wrench * wrench;

  state *= state;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state *= pose;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state *= twist;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state *= acc;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state *= wrench;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);

  pose *= state;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  pose *= pose;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  //pose *= twist;
  //pose *= acc;
  //pose *= wrench;

  //twist *= state;
  //twist *= pose;
  //twist *= twist;
  //twist *= acc;
  //twist *= wrench;

  //acc *= state;
  //acc *= pose;
  //acc *= twist;
  //acc *= acc;
  //acc *= wrench;

  //wrench *= state;
  //wrench *= pose;
  //wrench *= twist;
  //wrench *= acc;
  //wrench *= wrench;
}

TEST(CartesianStateTest, TestAdditionOperators) {
  CartesianState state = CartesianState::Random("test");
  CartesianPose pose = CartesianPose::Random("test");
  CartesianTwist twist = CartesianTwist::Random("test");
  CartesianAcceleration acc = CartesianAcceleration::Random("test");
  CartesianWrench wrench = CartesianWrench::Random("test");

  auto r1 = pose + pose;
  EXPECT_EQ(r1.get_type(), StateType::CARTESIAN_POSE);
  auto r2 = state + pose;
  EXPECT_EQ(r2.get_type(), StateType::CARTESIAN_STATE);
  auto r3 = pose + state;
  EXPECT_EQ(r3.get_type(), StateType::CARTESIAN_STATE);

  auto r4 = twist + twist;
  EXPECT_EQ(r4.get_type(), StateType::CARTESIAN_TWIST);
  auto r5 = state + twist;
  EXPECT_EQ(r5.get_type(), StateType::CARTESIAN_STATE);
  auto r6 = twist + state;
  EXPECT_EQ(r6.get_type(), StateType::CARTESIAN_STATE);

  auto r7 = acc + acc;
  EXPECT_EQ(r7.get_type(), StateType::CARTESIAN_ACCELERATION);
  auto r8 = state + acc;
  EXPECT_EQ(r8.get_type(), StateType::CARTESIAN_STATE);
  auto r9 = acc + state;
  EXPECT_EQ(r9.get_type(), StateType::CARTESIAN_STATE);

  auto r10 = wrench + wrench;
  EXPECT_EQ(r10.get_type(), StateType::CARTESIAN_WRENCH);
  auto r11 = state + wrench;
  EXPECT_EQ(r11.get_type(), StateType::CARTESIAN_STATE);
  auto r12 = wrench + state;
  EXPECT_EQ(r12.get_type(), StateType::CARTESIAN_STATE);

  // COMMENTED TEST BELOW EXPECTED TO BE NOT COMPILABLE

  //auto r = pose + twist;
  //auto r = pose + acc;
  //auto r = pose + wrench;

  //auto r = twist + pose;
  //auto r = twist + acc;
  //auto r = twist + wrench;

  //auto r = acc + pose;
  //auto r = acc + twist;
  //auto r = acc + wrench;

  state += state;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state += pose;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state += twist;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state += acc;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state += wrench;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);

  pose += state;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  pose += pose;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  //pose += twist;
  //pose += acc;
  //pose += wrench;

  twist += state;
  EXPECT_EQ(twist.get_type(), StateType::CARTESIAN_TWIST);
  twist += twist;
  EXPECT_EQ(twist.get_type(), StateType::CARTESIAN_TWIST);
  //twist += pose;
  //twist += acc;
  //twist += wrench;

  acc += state;
  EXPECT_EQ(acc.get_type(), StateType::CARTESIAN_ACCELERATION);
  acc += acc;
  EXPECT_EQ(acc.get_type(), StateType::CARTESIAN_ACCELERATION);
  //acc += pose;
  //acc += twist;
  //acc += wrench;

  wrench += state;
  EXPECT_EQ(wrench.get_type(), StateType::CARTESIAN_WRENCH);
  wrench += wrench;
  EXPECT_EQ(wrench.get_type(), StateType::CARTESIAN_WRENCH);
  //wrench += pose;
  //wrench += twist;
  //wrench += acc;
}

TEST(CartesianStateTest, TestSubtractionOperators) {
  CartesianState state = CartesianState::Random("test");
  CartesianPose pose = CartesianPose::Random("test");
  CartesianTwist twist = CartesianTwist::Random("test");
  CartesianAcceleration acc = CartesianAcceleration::Random("test");
  CartesianWrench wrench = CartesianWrench::Random("test");

  auto r1 = pose - pose;
  EXPECT_EQ(r1.get_type(), StateType::CARTESIAN_POSE);
  auto r2 = state - pose;
  EXPECT_EQ(r2.get_type(), StateType::CARTESIAN_STATE);
  auto r3 = pose - state;
  EXPECT_EQ(r3.get_type(), StateType::CARTESIAN_STATE);

  auto r4 = twist - twist;
  EXPECT_EQ(r4.get_type(), StateType::CARTESIAN_TWIST);
  auto r5 = state - twist;
  EXPECT_EQ(r5.get_type(), StateType::CARTESIAN_STATE);
  auto r6 = twist - state;
  EXPECT_EQ(r6.get_type(), StateType::CARTESIAN_STATE);

  auto r7 = acc - acc;
  EXPECT_EQ(r7.get_type(), StateType::CARTESIAN_ACCELERATION);
  auto r8 = state - acc;
  EXPECT_EQ(r8.get_type(), StateType::CARTESIAN_STATE);
  auto r9 = acc - state;
  EXPECT_EQ(r9.get_type(), StateType::CARTESIAN_STATE);

  auto r10 = wrench - wrench;
  EXPECT_EQ(r10.get_type(), StateType::CARTESIAN_WRENCH);
  auto r11 = state - wrench;
  EXPECT_EQ(r11.get_type(), StateType::CARTESIAN_STATE);
  auto r12 = wrench - state;
  EXPECT_EQ(r12.get_type(), StateType::CARTESIAN_STATE);

  // COMMENTED TEST BELOW EXPECTED TO BE NOT COMPILABLE

  //auto r = pose - twist;
  //auto r = pose - acc;
  //auto r = pose - wrench;

  //auto r = twist - pose;
  //auto r = twist - acc;
  //auto r = twist - wrench;

  //auto r = acc - pose;
  //auto r = acc - twist;
  //auto r = acc - wrench;

  state -= state;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state -= pose;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state -= twist;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state -= acc;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);
  state -= wrench;
  EXPECT_EQ(state.get_type(), StateType::CARTESIAN_STATE);

  pose -= state;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  pose -= pose;
  EXPECT_EQ(pose.get_type(), StateType::CARTESIAN_POSE);
  //pose -= twist;
  //pose -= acc;
  //pose -= wrench;

  twist -= state;
  EXPECT_EQ(twist.get_type(), StateType::CARTESIAN_TWIST);
  twist -= twist;
  EXPECT_EQ(twist.get_type(), StateType::CARTESIAN_TWIST);
  //twist -= pose;
  //twist -= acc;
  //twist -= wrench;

  acc -= state;
  EXPECT_EQ(acc.get_type(), StateType::CARTESIAN_ACCELERATION);
  acc -= acc;
  EXPECT_EQ(acc.get_type(), StateType::CARTESIAN_ACCELERATION);
  //acc -= pose;
  //acc -= twist;
  //acc -= wrench;

  wrench -= state;
  EXPECT_EQ(wrench.get_type(), StateType::CARTESIAN_WRENCH);
  wrench -= wrench;
  EXPECT_EQ(wrench.get_type(), StateType::CARTESIAN_WRENCH);
  //wrench -= pose;
  //wrench -= twist;
  //wrench -= acc;
}

TEST(CartesianStateTest, TestUtilities) {
  auto state_variable_type = string_to_cartesian_state_variable("position");
  EXPECT_EQ(state_variable_type, CartesianStateVariable::POSITION);
  EXPECT_EQ("position", cartesian_state_variable_to_string(CartesianStateVariable::POSITION));
  EXPECT_THROW(string_to_cartesian_state_variable("foo"), exceptions::InvalidStateVariableException);

  auto state = CartesianState();
  auto new_values = Eigen::VectorXd(4);
  new_values << 1.0, 2.0, 3.0, 4.0;
  EXPECT_THROW(state.set_state_variable(new_values, state_variable_type), exceptions::IncompatibleSizeException);
  new_values = Eigen::VectorXd(3);
  new_values << 1.0, 2.0, 3.0;
  state.set_state_variable(new_values, state_variable_type);
  EXPECT_TRUE(state.get_state_variable(CartesianStateVariable::POSITION).cwiseEqual(new_values).all());
  EXPECT_TRUE(state.get_state_variable(state_variable_type).cwiseEqual(new_values).all());

  new_values << 4.0, 5.0, 6.0;
  state.set_state_variable(new_values, CartesianStateVariable::POSITION);
  EXPECT_TRUE(state.get_position().cwiseEqual(new_values).all());
}
