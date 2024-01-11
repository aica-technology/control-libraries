#pragma once

#include <map>
#include <stdexcept>

namespace state_representation {

/**
 * @enum ParameterType
 * @brief The parameter value types.
 */
enum class ParameterType {
  BOOL,
  BOOL_ARRAY,
  INT,
  INT_ARRAY,
  DOUBLE,
  DOUBLE_ARRAY,
  STRING,
  STRING_ARRAY,
  STATE,
  VECTOR,
  MATRIX
};

static const std::map<std::string, ParameterType> parameter_type_map = {
    {"Bool", ParameterType::BOOL},     {"BoolArray", ParameterType::BOOL_ARRAY},
    {"Int", ParameterType::INT},       {"IntArray", ParameterType::INT_ARRAY},
    {"Double", ParameterType::DOUBLE}, {"DoubleArray", ParameterType::DOUBLE_ARRAY},
    {"String", ParameterType::STRING}, {"StringArray", ParameterType::STRING_ARRAY},
    {"State", ParameterType::STATE},   {"Vector", ParameterType::VECTOR},
    {"Matrix", ParameterType::MATRIX}};

/**
 * @brief Return the parameter type map, containing mappings of human-friendly strings for the type to their corresponding enum type
 * @return std::map<std::string, ParameterType> with label, ParameterType pairs
 */
[[maybe_unused]] static const std::map<std::string, ParameterType>& get_parameter_type_map() {
  return parameter_type_map;
}

/**
 * @brief Return the ParameterType that corresponds to a name
 * @param name The name for lookup
 * @throws std:out_of_range Exception if the name does not correspond to a ParameterType
 * @return ParameterType that corresponds to the (std::string) name
 */
[[maybe_unused]] static ParameterType parameter_type_from_name(const std::string& name) {
  try {
    return parameter_type_map.at(name);
  } catch (...) {
    throw std::out_of_range("The \"" + name + "\" ParameterType does not exist.");
  }
}

/**
 * @brief Convert parameter type enum to its corresponding human-readable name
 * @param parameter_type The parameter type
 * @throws std::out_of_range if the type does not exist
 * @return std::string of the corresponding parameter type in human-friendly form
 */
[[maybe_unused]] static std::string get_parameter_type_name(const ParameterType& parameter_type) {
  switch (parameter_type) {
    case ParameterType::BOOL:
      return "Bool";
    case ParameterType::BOOL_ARRAY:
      return "BoolArray";
    case ParameterType::INT:
      return "Int";
    case ParameterType::INT_ARRAY:
      return "IntArray";
    case ParameterType::DOUBLE:
      return "Double";
    case ParameterType::DOUBLE_ARRAY:
      return "DoubleArray";
    case ParameterType::STRING:
      return "String";
    case ParameterType::STRING_ARRAY:
      return "StringArray";
    case ParameterType::STATE:
      return "State";
    case ParameterType::VECTOR:
      return "Vector";
    case ParameterType::MATRIX:
      return "Matrix";
    default:
      throw std::out_of_range("This ParameterType does not exist.");
  }
}

}// namespace state_representation
