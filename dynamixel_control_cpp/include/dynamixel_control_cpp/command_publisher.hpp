#ifndef DYNAMIXEL_CONTROL_CPP_COMMAND_PUBLISHER_HPP_
#define DYNAMIXEL_CONTROL_CPP_COMMAND_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <enum.h>
#include <cmath>

using namespace dynamixel;

class DynamixelTeleop : public rclcpp::Node {
public:
    DynamixelTeleop();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void initialize_motors();
    void reset_torque();
    int to_twos_complement(int value, int byte_size);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    PortHandler *port_handler_;
    PacketHandler *packet_handler_;

    const std::string DEVICE_PORT = "/dev/ttyUSB0";
    const int BAUDRATE = 57600;
    const float PROTOCOL_VERSION = 2.0;

    const int SHOULDER1_ID = 1;
    const int SHOULDER2_ID = 2;
    const int ELBOW_ID = 3;
    const int WRIST1_ID = 5;
    const int WRIST2_ID = 4;

    const int ADDR_GOAL_VELOCITY = 104;
    const int ADDR_TORQUE_ENABLE = 64;
    const int ADDR_PRESENT_POSITION = 132;

    const int TORQUE_ENABLE = 1;
    const int TORQUE_DISABLE = 0;

    const int MAX_VELOCITY = 100;
    const int MIN_VELOCITY = -100;

    int velocity_wrist1_;
    int velocity_wrist2_;
    int velocity_elbow_;
    int velocity_shoulder1_;
    int velocity_shoulder2_;
};

#endif // DYNAMIXEL_CONTROL_CPP_COMMAND_PUBLISHER_HPP_