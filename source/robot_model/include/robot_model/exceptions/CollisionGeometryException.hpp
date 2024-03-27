#include <exception>
#include <string>

namespace robot_model::exceptions {
class CollisionGeometryException : public std::runtime_error {

public:
    explicit CollisionGeometryException(const std::string& error_message)
    : runtime_error("Collision geometry error: " + error_message) {}
};
} // namespace robot_model::exceptions
