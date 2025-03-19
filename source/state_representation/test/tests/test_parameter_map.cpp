#include <gtest/gtest.h>

#include "state_representation/exceptions/InvalidParameterException.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/parameters/ParameterMap.hpp"

using namespace state_representation;

class TestParameterMap : public ParameterMap {
public:
  explicit TestParameterMap(const ParameterInterfaceList& parameters) : ParameterMap(parameters) {}

  using ParameterMap::assert_parameter_valid;

  bool validate_called = false;

private:
  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override {
    validate_called = true;
    ParameterMap::validate_and_set_parameter(parameter);
  }
};

TEST(ParameterMapTest, ValidateSetParameter) {
  // The TestParameterMap overrides `validate_and_set_parameter`, setting parameters should update the flag
  auto map = TestParameterMap(ParameterInterfaceList());
  EXPECT_FALSE(map.validate_called);
  map.set_parameter(make_shared_parameter("int", 1));
  EXPECT_TRUE(map.validate_called);
  int value;
  EXPECT_NO_THROW(value = map.get_parameter_value<int>("int"));
  EXPECT_EQ(1, value);
  map.validate_called = false;
  map.set_parameter_value<int>("int", 2);
  EXPECT_TRUE(map.validate_called);
  EXPECT_NO_THROW(value = map.get_parameter("int")->get_parameter_value<int>());
  EXPECT_EQ(2, value);
  map.remove_parameter("int");
  EXPECT_THROW(map.remove_parameter("int"), exceptions::InvalidParameterException);
  EXPECT_THROW(value = map.get_parameter_value<int>("int"), exceptions::InvalidParameterException);
}

TEST(ParameterMapTest, AssertParameterValid) {
  ParameterInterfaceList params;
  params.push_back(std::make_shared<Parameter<std::string>>("string", "test"));
  params.push_back(std::make_shared<Parameter<JointState>>("joint", JointState::Zero("robot", 1)));

  auto map = TestParameterMap(params);
  // Parameter doesn't exist
  EXPECT_THROW(map.assert_parameter_valid(make_shared_parameter("int", 1)), exceptions::InvalidParameterException);
  // Parameter has incorrect parameter type
  EXPECT_THROW(map.assert_parameter_valid(make_shared_parameter("string", 1)), exceptions::InvalidParameterException);
  // Parameter has incorrect parameter state type
  EXPECT_THROW(
      map.assert_parameter_valid(make_shared_parameter("joint", CartesianState::Random("test"))),
      exceptions::InvalidParameterException
  );
}
