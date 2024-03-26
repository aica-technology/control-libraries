#include <exception>
#include <string>

namespace robot_model::exceptions {
class CollisionGeometryException : public std::invalid_argument {
private:
    std::string robot_name;

public:
    explicit CollisionGeometryException(const std::string& robot_name)
    : invalid_argument("Geometry meshes of the robot are not loaded for robot: " + robot_name), robot_name(robot_name) {}

    // Optionally, provide a method to get the robot name
    std::string get_robot_name() const { return robot_name; }
};
} // namespace robot_model::exceptions
