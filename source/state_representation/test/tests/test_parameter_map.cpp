#include <gtest/gtest.h>

#include "state_representation/parameters/ParameterMap.hpp"
#include "state_representation/exceptions/InvalidParameterException.hpp"

using namespace state_representation;

class TestParameterMap : public ParameterMap {
public:
  explicit TestParameterMap(const ParameterInterfaceList& parameters) : ParameterMap(parameters) {}

private:
  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override {
    assert_parameter_valid(parameter);
    this->parameters_.insert_or_assign(parameter->get_name(), parameter);
  }
};

TEST(ParameterMapTest, ParameterMap) {
  auto map = ParameterMap();
  map.set_parameter(make_shared_parameter("int", 1));
  int value;
  EXPECT_NO_THROW(value = map.get_parameter_value<int>("int"));
  EXPECT_EQ(1, value);
  map.set_parameter_value<int>("int", 2);
  EXPECT_NO_THROW(value = map.get_parameter("int")->get_parameter_value<int>());
  EXPECT_EQ(2, value);
  map.remove_parameter("int");
  EXPECT_THROW(map.remove_parameter("int"), exceptions::InvalidParameterException);
  EXPECT_THROW(value = map.get_parameter_value<int>("int"), exceptions::InvalidParameterException);
}

TEST(ParameterMapTest, DerivedParameterMap) {
  ParameterInterfaceList params;
  params.push_back(std::make_shared<Parameter<std::string>>("string", "test"));
  params.push_back(std::make_shared<Parameter<JointState>>("joint", JointState::Zero("robot", 1)));

  auto map = TestParameterMap(params);
  // Parameter that doesn't exist cannot be set
  EXPECT_THROW(map.set_parameter_value("int", 1), exceptions::InvalidParameterException);

  EXPECT_NO_THROW(map.set_parameter_value<std::string>("string", "new"));
  // Parameter cannot be set with incorrect type
  EXPECT_THROW(map.set_parameter_value("string", 1), exceptions::InvalidParameterException);
  // Parameter cannot be set with incorrect state type
  EXPECT_THROW(map.set_parameter_value("joint", CartesianState::Random("test")), exceptions::InvalidParameterException);
}
