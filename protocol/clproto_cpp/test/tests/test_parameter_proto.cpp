#include <gtest/gtest.h>

#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "clproto.hpp"
#include "test_encode_decode.hpp"

using namespace state_representation;

template<typename T> using ParamT = std::tuple<T, clproto::ParameterMessageType>;

static std::tuple<ParamT<bool>,
                  ParamT<std::vector<bool>>,
                  ParamT<int>,
                  ParamT<std::vector<int>>,
                  ParamT<double>,
                  ParamT<std::vector<double>>,
                  ParamT<std::string>,
                  ParamT<std::vector<std::string>>,
                  ParamT<Eigen::VectorXd>,
                  ParamT<Eigen::MatrixXd>> parameter_test_cases{
    std::make_tuple(false, clproto::ParameterMessageType::BOOL),
    std::make_tuple(std::vector<bool>({true, false, true}), clproto::ParameterMessageType::BOOL_ARRAY),
    std::make_tuple(1, clproto::ParameterMessageType::INT),
    std::make_tuple(std::vector<int>({1, 2, 3}), clproto::ParameterMessageType::INT_ARRAY),
    std::make_tuple(1.0, clproto::ParameterMessageType::DOUBLE), std::make_tuple(
        std::vector<double>({1.0, 2.0, 3.0}), clproto::ParameterMessageType::DOUBLE_ARRAY
    ), std::make_tuple("test", clproto::ParameterMessageType::STRING), std::make_tuple(
        std::vector<std::string>({"1", "2", "3"}), clproto::ParameterMessageType::STRING_ARRAY
    ), std::make_tuple(Eigen::VectorXd::Random(2), clproto::ParameterMessageType::VECTOR),
    std::make_tuple(Eigen::MatrixXd::Random(2, 2), clproto::ParameterMessageType::MATRIX)};

template<typename T>
static void test_parameter_equal(const T& send_state, const T& recv_state) {
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_EQ(send_state.is_empty(), recv_state.is_empty());
  if (send_state) {
    EXPECT_EQ(send_state.get_value(), recv_state.get_value());
  }
}

template<typename T>
class ParameterProtoTest : public testing::Test {
public:
  ParameterProtoTest() : test_case_{std::get<ParamT<T>>(parameter_test_cases)} {}
protected:
  ParamT<T> test_case_;
};
TYPED_TEST_SUITE_P(ParameterProtoTest);

TEST(ParameterProtoTest, EncodeParameterInterface) {
  auto send_state_ptr = make_shared_state(ParameterInterface("name", ParameterType::BOOL));
  EXPECT_THROW(std::string msg = clproto::encode(send_state_ptr), std::invalid_argument);
}

TYPED_TEST_P(ParameterProtoTest, EncodeDecodeParameter) {
  auto param = Parameter<TypeParam>("test", std::get<0>(this->test_case_));
  clproto::test_encode_decode<Parameter<TypeParam>>(
      param, clproto::PARAMETER_MESSAGE, test_parameter_equal<Parameter<TypeParam>>, std::get<1>(this->test_case_));
  param.reset();
  clproto::test_encode_decode<Parameter<TypeParam>>(
      param, clproto::PARAMETER_MESSAGE, test_parameter_equal<Parameter<TypeParam>>, std::get<1>(this->test_case_));
}

TEST(ParameterProtoTest, EncodeDecodeInvalidParameter) {
  auto send_state = Parameter<CartesianState>("state", CartesianState::Random("state"));
  auto send_state_ptr = make_shared_state(send_state);
  EXPECT_THROW(clproto::encode(send_state_ptr), std::invalid_argument);

  auto send_state_2 = State("A");
  std::string msg = clproto::encode(send_state_2);

  Parameter<CartesianState> recv_state("");
  auto recv_state_ptr = make_shared_state(recv_state);
  EXPECT_FALSE(clproto::decode(msg, recv_state_ptr));
}

REGISTER_TYPED_TEST_SUITE_P(ParameterProtoTest, EncodeDecodeParameter);

using ParameterTestTypes = testing::Types<bool,
                                          std::vector<bool>,
                                          int,
                                          std::vector<int>,
                                          double,
                                          std::vector<double>,
                                          std::string,
                                          std::vector<std::string>,
                                          Eigen::VectorXd,
                                          Eigen::MatrixXd>;
INSTANTIATE_TYPED_TEST_SUITE_P(Type, ParameterProtoTest, ParameterTestTypes);
