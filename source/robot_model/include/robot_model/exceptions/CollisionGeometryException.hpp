#include <exception>
#include <string>

namespace robot_model::exceptions {
class CollisionGeometryException : public std::invalid_argument {

public:
    explicit CollisionGeometryException(const std::string& error_message)
    : invalid_argument("Error in CollisionGeomerty: " + error_message) {}
};
} // namespace robot_model::exceptions
