#include "state_representation/IOState.hpp"

namespace state_representation {

template<>
void IOState<double>::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

template<>
void IOState<bool>::set_data(const std::vector<bool>& data) {
  Eigen::Vector<bool, Eigen::Dynamic> vec;
  vec.resize(data.size());
  for (unsigned int i = 0; i < data.size(); ++i) {
    vec(i) = data.at(i);
  }
  this->set_data(vec);
}

template<>
std::vector<double> IOState<double>::to_std_vector() const {
  return {this->data_.data(), this->data_.data() + this->data_.size()};
}

template<>
std::vector<bool> IOState<bool>::to_std_vector() const {
  std::vector<bool> vec;
  vec.resize(this->get_size());
  for (unsigned int i = 0; i < this->get_size(); ++i) {
    vec.at(i) = this->data_(i);
  }
  return vec;
}
}// namespace state_representation
