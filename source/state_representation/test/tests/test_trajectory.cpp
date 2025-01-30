#include <chrono>
#include <gtest/gtest.h>
#include <stdexcept>
#include <type_traits>

#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/trajectory/JointTrajectory.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"

using namespace state_representation;

class TrajectoryBaseInterface : public TrajectoryBase<TrajectoryPoint> {
public:
  using TrajectoryBase<TrajectoryPoint>::TrajectoryBase;
  using TrajectoryBase<TrajectoryPoint>::add_point;
  using TrajectoryBase<TrajectoryPoint>::add_points;
  using TrajectoryBase<TrajectoryPoint>::set_point;
  using TrajectoryBase<TrajectoryPoint>::set_points;
  using TrajectoryBase<TrajectoryPoint>::get_point;
  using TrajectoryBase<TrajectoryPoint>::get_points;
  using TrajectoryBase<TrajectoryPoint>::insert_point;
  using TrajectoryBase<TrajectoryPoint>::operator[];

  TrajectoryBaseInterface() : TrajectoryBase<TrajectoryPoint>() {}
  TrajectoryBaseInterface(const std::string& name) : TrajectoryBase<TrajectoryPoint>(name) {}
};

template<typename TrajectoryType>
class TrajectoryTest : public testing::Test {
public:
  template<typename PointType>
  void add_point(PointType point, std::chrono::nanoseconds duration = std::chrono::nanoseconds(100)) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      this->trajectory->add_point(point);
    } else {
      this->trajectory->add_point(point, duration);
    }
  }

  template<typename PointType>
  void add_points(std::vector<PointType>& points, std::vector<std::chrono::nanoseconds>& durations) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      this->trajectory->add_points(points);
    } else {
      this->trajectory->add_points(points, durations);
    }
  }

  template<typename PointType>
  void
  insert_point(PointType point, unsigned int index, std::chrono::nanoseconds duration = std::chrono::nanoseconds(100)) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      this->trajectory->insert_point(point, index);
    } else {
      this->trajectory->insert_point(point, duration, index);
    }
  }

  template<typename PointType>
  void
  set_point(PointType point, unsigned int index, std::chrono::nanoseconds duration = std::chrono::nanoseconds(100)) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      this->trajectory->set_point(point, index);
    } else {
      this->trajectory->set_point(point, duration, index);
    }
  }

  template<typename PointType>
  void set_points(std::vector<PointType>& points, std::vector<std::chrono::nanoseconds>& durations) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      this->trajectory->set_points(points);
    } else {
      this->trajectory->set_points(points, durations);
    }
  }

  void delete_point() { this->trajectory->delete_point(); }

  void delete_point_index(unsigned int index) { this->trajectory->delete_point(index); }

  template<typename PointType>
  void
  expect_equal(PointType point, unsigned int index, std::chrono::nanoseconds duration = std::chrono::nanoseconds(100)) {
    if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
      EXPECT_EQ(this->trajectory->operator[](index).data, point.data);
      EXPECT_EQ(this->trajectory->operator[](index).duration, point.duration);
    } else {
      EXPECT_EQ(this->trajectory->operator[](index).first.data(), point.data());
      EXPECT_EQ(this->trajectory->operator[](index).first.get_name(), point.get_name());
      if constexpr (std::is_same_v<PointType, CartesianState>) {
        EXPECT_EQ(this->trajectory->operator[](index).first.get_reference_frame(), point.get_reference_frame());
      } else if constexpr (std::is_same_v<PointType, JointState>) {
        EXPECT_EQ(this->trajectory->operator[](index).first.get_names(), point.get_names());
      }
      EXPECT_EQ(this->trajectory->operator[](index).second, duration);
    }
  }

  std::shared_ptr<TrajectoryType> trajectory;
};
TYPED_TEST_SUITE_P(TrajectoryTest);

TEST(TrajectoryTest, ConstructTrajectory) {
  // Base class
  EXPECT_NO_THROW(TrajectoryBaseInterface trajectory);
  EXPECT_NO_THROW(TrajectoryBaseInterface trajectory("foo"));

  // Cartesian trajectory
  EXPECT_NO_THROW(CartesianTrajectory trajectory("foo"));
  EXPECT_NO_THROW(CartesianTrajectory trajectory("foo", CartesianState::Random("foo"), std::chrono::nanoseconds(100)));
  EXPECT_NO_THROW(
      CartesianTrajectory trajectory("foo", {CartesianState::Random("foo")}, {std::chrono::nanoseconds(100)})
  );

  EXPECT_THROW(
      CartesianTrajectory trajectory("foo", CartesianState(), std::chrono::nanoseconds(100)),
      exceptions::EmptyStateException
  );
  EXPECT_THROW(
      CartesianTrajectory trajectory(
          "foo", {CartesianState::Random("foo", "some_world"), CartesianState::Random("foo")},
          {std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)}
      ),
      exceptions::IncompatibleReferenceFramesException
  );
  EXPECT_THROW(
      CartesianTrajectory trajectory(
          "foo", {CartesianState::Random("foo"), CartesianState::Random("foo")}, {std::chrono::nanoseconds(100)}
      ),
      exceptions::IncompatibleSizeException
  );

  // Joint trajectory
  EXPECT_NO_THROW(JointTrajectory trajectory("foo"));
  EXPECT_NO_THROW(JointTrajectory trajectory("foo", JointState::Random("foo", 25), std::chrono::nanoseconds(100)));
  EXPECT_NO_THROW(JointTrajectory trajectory("foo", {JointState::Random("foo", 25)}, {std::chrono::nanoseconds(100)}));

  EXPECT_THROW(
      JointTrajectory trajectory("foo", JointState(), std::chrono::nanoseconds(100)), exceptions::EmptyStateException
  );
  EXPECT_THROW(
      JointTrajectory trajectory(
          "foo", {JointState::Random("foo", 25), JointState::Random("foo", 24)},
          {std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)}
      ),
      exceptions::IncompatibleStatesException
  );
  EXPECT_THROW(
      JointTrajectory trajectory(
          "foo", {JointState::Random("foo", 25), JointState::Random("foo", 25)}, {std::chrono::nanoseconds(100)}
      ),
      exceptions::IncompatibleSizeException
  );
}

TYPED_TEST_P(TrajectoryTest, AddRemovePoints) {
  EXPECT_NO_THROW(this->trajectory = std::make_shared<TypeParam>("trajectory"));
  EXPECT_EQ(this->trajectory->get_size(), 0);

  using PointType = typename std::conditional<
      std::is_same_v<TypeParam, TrajectoryBaseInterface>, TrajectoryPoint,
      typename std::conditional<std::is_same_v<CartesianTrajectory, TypeParam>, CartesianState, JointState>::type>::
      type;

  PointType point0;
  PointType point1;
  PointType point2;
  PointType point3;

  if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
    point0.data = Eigen::VectorXd::Random(3);
    point0.duration = std::chrono::nanoseconds(10);
    point1.data = Eigen::VectorXd::Random(3);
    point1.duration = std::chrono::nanoseconds(20);
    point2.data = Eigen::VectorXd::Random(3);
    point2.duration = std::chrono::nanoseconds(30);
    point3.data = Eigen::VectorXd::Random(3);
    point3.duration = std::chrono::nanoseconds(40);
  } else if constexpr (std::is_same_v<PointType, CartesianState>) {
    point0 = CartesianState::Random("foo");
    point1 = CartesianState::Random("bar");
    point2 = CartesianState::Random("baz");
    point3 = CartesianState::Random("qux");
  } else if constexpr (std::is_same_v<PointType, JointState>) {
    point0 = JointState::Random("foo", 25);
    point1 = JointState::Random("bar", 25);
    point2 = JointState::Random("baz", 25);
    point3 = JointState::Random("qux", 25);
    this->trajectory->set_joint_names(point0.get_names());
  }

  // additions and insertions of single points
  EXPECT_NO_THROW(this->add_point(point0));
  EXPECT_EQ(this->trajectory->get_size(), 1);
  this->expect_equal(point0, 0);
  EXPECT_NO_THROW(this->add_point(point2));
  EXPECT_EQ(this->trajectory->get_size(), 2);
  this->expect_equal(point2, 1);
  EXPECT_NO_THROW(this->insert_point(point1, 1));
  EXPECT_EQ(this->trajectory->get_size(), 3);
  this->expect_equal(point1, 1);
  this->expect_equal(point2, 2);
  EXPECT_EQ(this->trajectory->get_durations().size(), 3);
  this->set_point(point3, 1);
  this->expect_equal(point3, 1);

  // deletions
  EXPECT_NO_THROW(this->delete_point_index(1));
  this->expect_equal(point2, 1);
  EXPECT_NO_THROW(this->delete_point());
  this->expect_equal(point0, 0);
  EXPECT_NO_THROW(this->delete_point());
  EXPECT_EQ(this->trajectory->get_size(), 0);

  // additons and insertions of multiple points
  std::vector<PointType> points = {point0, point1, point2};
  std::vector<std::chrono::nanoseconds> durations = {
      std::chrono::nanoseconds(10), std::chrono::nanoseconds(20), std::chrono::nanoseconds(30)
  };
  EXPECT_NO_THROW(this->add_points(points, durations));
  for (unsigned int i = 0; i < this->trajectory->get_size(); ++i) {
    this->expect_equal(points[i], i, durations[i]);
  }
  std::vector<PointType> shuffled_points = {point2, point0, point1};
  std::vector<std::chrono::nanoseconds> shuffled_durations = {
      std::chrono::nanoseconds(30), std::chrono::nanoseconds(10), std::chrono::nanoseconds(20)
  };
  EXPECT_NO_THROW(this->set_points(shuffled_points, shuffled_durations));
  for (unsigned int i = 0; i < this->trajectory->get_size(); ++i) {
    this->expect_equal(shuffled_points[i], i, shuffled_durations[i]);
  }
}

TYPED_TEST_P(TrajectoryTest, Exceptions) {
  EXPECT_NO_THROW(this->trajectory = std::make_shared<TypeParam>("trajectory"));
  EXPECT_EQ(this->trajectory->get_size(), 0);

  using PointType = typename std::conditional<
      std::is_same_v<TypeParam, TrajectoryBaseInterface>, TrajectoryPoint,
      typename std::conditional<std::is_same_v<CartesianTrajectory, TypeParam>, CartesianState, JointState>::type>::
      type;

  PointType point0;
  PointType point1;
  PointType point2;
  PointType point3;

  if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
    point0.data = Eigen::VectorXd::Random(3);
    point0.duration = std::chrono::nanoseconds(10);
    point1.data = Eigen::VectorXd::Random(3);
    point1.duration = std::chrono::nanoseconds(20);
    point2.data = Eigen::VectorXd::Random(3);
    point2.duration = std::chrono::nanoseconds(30);
    point3.data = Eigen::VectorXd::Random(3);
    point3.duration = std::chrono::nanoseconds(40);
  } else if constexpr (std::is_same_v<PointType, CartesianState>) {
    point0 = CartesianState::Random("foo");
    point1 = CartesianState::Random("bar");
    point2 = CartesianState::Random("baz");
    point3 = CartesianState::Random("qux", "some_other_world");
  } else if constexpr (std::is_same_v<PointType, JointState>) {
    point0 = JointState::Random("foo", 25);
    point1 = JointState::Random("bar", 25);
    point2 = JointState::Random("baz", 25);
    point3 = JointState::Random("qux", 20);
    this->trajectory->set_joint_names(point0.get_names());
  }

  std::vector<PointType> points = {point0, point1, point2};
  std::vector<std::chrono::nanoseconds> durations = {
      std::chrono::nanoseconds(10), std::chrono::nanoseconds(20), std::chrono::nanoseconds(30)
  };
  EXPECT_NO_THROW(this->add_points(points, durations));

  EXPECT_THROW(this->delete_point_index(10), std::out_of_range);

  points.push_back(point1);
  durations.push_back(std::chrono::nanoseconds(40));
  EXPECT_THROW(this->set_points(points, durations), exceptions::IncompatibleSizeException);

  points.pop_back();
  durations.pop_back();
  points[2] = point3;
  if constexpr (std::is_same_v<PointType, CartesianState>) {
    EXPECT_THROW(this->set_points(points, durations), exceptions::IncompatibleReferenceFramesException);
    EXPECT_THROW(this->set_point(point3, 1), exceptions::IncompatibleReferenceFramesException);
    EXPECT_THROW(this->add_point(point3), exceptions::IncompatibleReferenceFramesException);
    EXPECT_THROW(this->add_point(CartesianState()), exceptions::EmptyStateException);
  } else if constexpr (std::is_same_v<PointType, JointState>) {
    EXPECT_THROW(this->set_points(points, durations), exceptions::IncompatibleStatesException);
    EXPECT_THROW(this->set_point(point3, 1), exceptions::IncompatibleStatesException);
    EXPECT_THROW(this->add_point(point3), exceptions::IncompatibleStatesException);
    EXPECT_THROW(this->add_point(JointState()), exceptions::EmptyStateException);
  }

  EXPECT_THROW(this->set_point(point1, 10), std::out_of_range);
  EXPECT_THROW(this->insert_point(point1, 10), std::out_of_range);
  EXPECT_THROW(this->trajectory->get_point(10), std::out_of_range);
  EXPECT_THROW(this->trajectory->operator[](10), std::out_of_range);
}

TYPED_TEST_P(TrajectoryTest, Getters) {
  EXPECT_NO_THROW(this->trajectory = std::make_shared<TypeParam>("trajectory"));
  EXPECT_EQ(this->trajectory->get_size(), 0);

  using PointType = typename std::conditional<
      std::is_same_v<TypeParam, TrajectoryBaseInterface>, TrajectoryPoint,
      typename std::conditional<std::is_same_v<CartesianTrajectory, TypeParam>, CartesianState, JointState>::type>::
      type;

  PointType point0;
  PointType point1;
  PointType point2;

  if constexpr (std::is_same_v<PointType, TrajectoryPoint>) {
    point0.data = Eigen::VectorXd::Random(3);
    point0.duration = std::chrono::nanoseconds(10);
    point1.data = Eigen::VectorXd::Random(3);
    point1.duration = std::chrono::nanoseconds(20);
    point2.data = Eigen::VectorXd::Random(3);
    point2.duration = std::chrono::nanoseconds(30);
  } else if constexpr (std::is_same_v<PointType, CartesianState>) {
    point0 = CartesianState::Random("foo");
    point1 = CartesianState::Random("bar");
    point2 = CartesianState::Random("baz");
  } else if constexpr (std::is_same_v<PointType, JointState>) {
    point0 = JointState::Random("foo", 25);
    point1 = JointState::Random("bar", 25);
    point2 = JointState::Random("baz", 25);
    this->trajectory->set_joint_names(point0.get_names());
  }

  std::vector<PointType> points = {point0, point1, point2};
  std::vector<std::chrono::nanoseconds> durations = {
      std::chrono::nanoseconds(10), std::chrono::nanoseconds(20), std::chrono::nanoseconds(30)
  };
  EXPECT_NO_THROW(this->add_points(points, durations));
  auto trajectory_points = this->trajectory->get_points();
  auto trajectory_durations = this->trajectory->get_durations();
  for (unsigned int i = 0; i < this->trajectory->get_size(); ++i) {
    EXPECT_NO_THROW(this->expect_equal(points[i], i, durations[i]));
    EXPECT_NO_THROW(this->expect_equal(trajectory_points[i], i, durations[i]));
    EXPECT_NO_THROW(this->expect_equal(this->trajectory->get_point(i), i, trajectory_durations[i]));
  }

  auto times_from_start = this->trajectory->get_times_from_start();
  std::chrono::nanoseconds time_from_start = std::chrono::nanoseconds(0);
  for (unsigned int i = 0; i < this->trajectory->get_size(); ++i) {
    time_from_start += durations[i];
    EXPECT_EQ(times_from_start[i], time_from_start);
  }

  this->trajectory->reset();
  EXPECT_EQ(this->trajectory->get_size(), 0);
  EXPECT_EQ(this->trajectory->get_durations().size(), 0);
  if constexpr (std::is_same_v<PointType, CartesianState>) {
    EXPECT_STREQ(this->trajectory->get_reference_frame().c_str(), "world");
  } else if constexpr (std::is_same_v<PointType, JointState>) {
    EXPECT_NE(this->trajectory->get_joint_names().size(), 0);
  }
  EXPECT_TRUE(this->trajectory->is_empty());
}

REGISTER_TYPED_TEST_SUITE_P(TrajectoryTest, AddRemovePoints, Exceptions, Getters);

using TrajectoryTypes = testing::Types<TrajectoryBaseInterface, CartesianTrajectory, JointTrajectory>;
INSTANTIATE_TYPED_TEST_SUITE_P(Type, TrajectoryTest, TrajectoryTypes);
