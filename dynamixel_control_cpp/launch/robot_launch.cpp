#include <memory>
#include "launch/launch.hpp"
#include "launch_ros/actions/node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace launch;
using namespace launch_ros;

LaunchDescription generate_launch_description() {
    return LaunchDescription({
        Node(
            "joy_linux_node",
            "joy_linux",
            {
                {"controller_id", 0},
                {"deadzone", 0.1},
                {"sensitivity", 1.0}
            },
            {
                {"/joy", "/controller1/joy"}
            }
        ),
        Node(
            "command_publisher",
            "dynamixel_control_cpp",
            {
                {"controller_id", 1},
                {"deadzone", 0.1},
                {"sensitivity", 1.0}
            },
            {
                {"/joy", "/controller1/joy"}
            }
        )
    });
}