#include <fstream>
#include <gtest/gtest.h>
#include "state_representation/AnalogIOState.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/exceptions/IONotFoundException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

TEST(AnalogIOStateTest, Constructors) {
  AnalogIOState empty;
  EXPECT_EQ(empty.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(empty.get_name(), "");
  EXPECT_TRUE(empty.is_empty());
  EXPECT_EQ(empty.get_size(), 0);
  EXPECT_THROW(empty.data(), exceptions::EmptyStateException);

  AnalogIOState io1("test", 3);
  EXPECT_EQ(io1.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(io1.get_name(), "test");
  EXPECT_TRUE(io1.is_empty());
  EXPECT_EQ(io1.get_size(), 3);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(io1.get_names().at(i), "io" + std::to_string(i));
  }
  EXPECT_THROW(io1.data(), exceptions::EmptyStateException);

  std::vector<std::string> io_names{"io_10", "io_20"};
  AnalogIOState io2("test", io_names);
  EXPECT_EQ(io2.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(io2.get_name(), "test");
  EXPECT_TRUE(io2.is_empty());
  EXPECT_EQ(io2.get_size(), io_names.size());
  for (std::size_t i = 0; i < io_names.size(); ++i) {
    EXPECT_EQ(io2.get_names().at(i), io_names.at(i));
  }
  EXPECT_THROW(io2.data(), exceptions::EmptyStateException);
}

TEST(AnalogIOStateTest, ZeroInitialization) {
  AnalogIOState zero = AnalogIOState::Zero("test", 3);
  EXPECT_EQ(zero.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_FALSE(zero.is_empty());
  EXPECT_EQ(zero.data().norm(), 0);

  AnalogIOState zero2 = AnalogIOState::Zero("test", std::vector<std::string>{"0", "1"});
  EXPECT_EQ(zero2.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_FALSE(zero2.is_empty());
  EXPECT_EQ(zero2.data().norm(), 0);
}

TEST(AnalogIOStateTest, RandomStateInitialization) {
  AnalogIOState random = AnalogIOState::Random("test", 3);
  EXPECT_EQ(random.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_NE(random.data().norm(), 0);

  AnalogIOState random2 = AnalogIOState::Random("test", std::vector<std::string>{"0", "1"});
  EXPECT_EQ(random2.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_NE(random2.data().norm(), 0);
}

TEST(AnalogIOStateTest, CopyConstructor) {
  AnalogIOState random = AnalogIOState::Random("test", 3);
  AnalogIOState copy1(random);
  EXPECT_EQ(copy1.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(random.get_name(), copy1.get_name());
  EXPECT_EQ(random.get_names(), copy1.get_names());
  EXPECT_EQ(random.get_size(), copy1.get_size());
  EXPECT_EQ(random.data(), copy1.data());

  AnalogIOState copy2 = random;
  EXPECT_EQ(copy2.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(random.get_name(), copy2.get_name());
  EXPECT_EQ(random.get_names(), copy2.get_names());
  EXPECT_EQ(random.get_size(), copy2.get_size());
  EXPECT_EQ(random.data(), copy2.data());

  AnalogIOState copy3 = random.copy();
  EXPECT_EQ(copy3.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(random.get_name(), copy3.get_name());
  EXPECT_EQ(random.get_names(), copy3.get_names());
  EXPECT_EQ(random.get_size(), copy3.get_size());
  EXPECT_EQ(random.data(), copy3.data());

  AnalogIOState empty;
  AnalogIOState copy4(empty);
  EXPECT_EQ(copy4.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_TRUE(copy4.is_empty());
  AnalogIOState copy5 = empty;
  EXPECT_EQ(copy5.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_TRUE(copy5.is_empty());
  AnalogIOState copy6 = empty.copy();
  EXPECT_EQ(copy6.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_TRUE(copy6.is_empty());
}

TEST(AnalogIOStateTest, GetSetFields) {
  AnalogIOState io("test", 3);

  // name
  io.set_name("io");
  EXPECT_EQ(io.get_name(), "io");
  EXPECT_EQ(io.get_size(), 3);
  std::vector<std::string> io_names{"1", "2", "3"};
  io.set_names(io_names);
  for (std::size_t i = 0; i < io_names.size(); ++i) {
    EXPECT_EQ(io.get_names().at(i), io_names.at(i));
  }
  io_names.emplace_back("j4");
  EXPECT_THROW(io.set_names(4), exceptions::IncompatibleSizeException);
  EXPECT_THROW(io.set_names(io_names), exceptions::IncompatibleSizeException);

  // fields
  std::vector<double> data{1, 2, 3};
  io.set_data(data);
  for (std::size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(io.data()(i), data.at(i));
    EXPECT_EQ(io.get_value(i), data.at(i));
    EXPECT_EQ(io.get_value(io.get_names().at(i)), data.at(i));
  }
  EXPECT_THROW(io.get_value(io.get_size() + 1), exceptions::IONotFoundException);
  EXPECT_THROW(io.get_value("test"), exceptions::IONotFoundException);

  io.set_zero();
  EXPECT_EQ(io.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(io.data().norm(), 0);
  EXPECT_EQ(io.is_empty(), false);
  io.reset();
  EXPECT_THROW(io.data(), exceptions::EmptyStateException);
  EXPECT_EQ(io.is_empty(), true);
}

TEST(AnalogIOStateTest, GetSetField) {
  AnalogIOState io("test", 3);

  // fields
  io.set_value(1.0, "io0");
  io.set_value(1.1, 1);
  EXPECT_EQ(io.get_value(0), 1.0);
  EXPECT_EQ(io.get_value(1), 1.1);
  EXPECT_EQ(io.get_value(2), 0);
  EXPECT_THROW(io.set_value(1, io.get_size() + 1), exceptions::IONotFoundException);
  EXPECT_THROW(io.set_value(1, "test"), exceptions::IONotFoundException);
}

TEST(AnalogIOStateTest, Compatibility) {
  AnalogIOState io1("test", 3);
  AnalogIOState io2("test", std::vector<std::string>{"1", "2", "3"});
  AnalogIOState io3("test", 4);
  AnalogIOState io4("io", 3);

  EXPECT_TRUE(io1.is_incompatible(io2));
  EXPECT_TRUE(io1.is_incompatible(io3));
  EXPECT_FALSE(io1.is_incompatible(io4));
}

TEST(AnalogIOStateTest, SetZero) {
  AnalogIOState random1 = AnalogIOState::Random("test", 3);
  random1.reset();
  EXPECT_THROW(random1.data(), exceptions::EmptyStateException);

  AnalogIOState random2 = AnalogIOState::Random("test", 3);
  random2.set_zero();
  EXPECT_EQ(random2.get_type(), StateType::ANALOG_IO_STATE);
  EXPECT_EQ(random2.data().norm(), 0);
}

TEST(AnalogIOStateTest, GetSetData) {
  AnalogIOState io1 = AnalogIOState::Zero("test", 3);
  AnalogIOState io2 = AnalogIOState::Random("test", 3);

  io1.set_data(io2.data());
  EXPECT_TRUE(io2.data().isApprox(io1.data()));

  auto state_vec = io2.to_std_vector();
  io1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), io1.data()(i));
  }
  EXPECT_THROW(io1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(AnalogIOStateTest, GetIndexByName) {
  AnalogIOState io = AnalogIOState::Random("test", 3);
  for (std::size_t i = 0; i < io.get_size(); ++i) {
    auto index = io.get_io_index("io" + std::to_string(i));
    EXPECT_EQ(index, i);
  }
  EXPECT_THROW(io.get_io_index("io5"), exceptions::IONotFoundException);
}

TEST(AnalogIOStateTest, Truthiness) {
  AnalogIOState empty("test", 1);
  EXPECT_TRUE(empty.is_empty());
  EXPECT_FALSE(empty);

  empty.set_data(Eigen::VectorXd::Random(1));
  EXPECT_FALSE(empty.is_empty());
  EXPECT_TRUE(empty);
}
