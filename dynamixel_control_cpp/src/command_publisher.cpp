#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cmath>

using namespace dynamixel;

class DynamixelTeleop : public rclcpp::Node {
public:
    DynamixelTeleop() : Node("dynamixel_teleop") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&DynamixelTeleop::joy_callback, this, std::placeholders::_1));


    port_handler_ = PortHandler::getPortHandler(DEVICE_PORT);
    packet_handler_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port");
            return;
        }
        if (!port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
            return;
        }

        enableTorque();
        RCLCPP_INFO(this->get_logger(), "Dynamixel ready. Use joystick to control.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        // Process joystick input and send commands to motors
        // Similar logic as in the Python version
    }

    void enableTorque() {
        // Enable torque for all motors
        writeTorque(WRIST1_ID, TORQUE_ENABLE);
        writeTorque(WRIST2_ID, TORQUE_ENABLE);
        writeTorque(ELBOW_ID, TORQUE_ENABLE);
        writeTorque(SHOULDER1_ID, TORQUE_ENABLE);
        writeTorque(SHOULDER2_ID, TORQUE_ENABLE);
    }

    void writeTorque(int motor_id, int torque) {
        int dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, motor_id, ADDR_TORQUE_ENABLE, torque);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write torque: %s", packet_handler_->getTxRxResult(dxl_comm_result));
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    PortHandler *port_handler_;
    PacketHandler *packet_handler_;

    const char *DEVICE_PORT = "/dev/ttyUSB0";
    const int BAUDRATE = 57600;
    const float PROTOCOL_VERSION = 2.0;

    const int WRIST1_ID = 1;
    const int WRIST2_ID = 2;
    const int ELBOW_ID = 3;
    const int SHOULDER1_ID = 4;
    const int SHOULDER2_ID = 5;
    const int ADDR_TORQUE_ENABLE = 64;
    const int TORQUE_ENABLE = 1;
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelTeleop>());
    rclcpp::shutdown();
    return 0;
}