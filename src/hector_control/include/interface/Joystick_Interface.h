#pragma once
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <sensor_msgs/msg/joy.hpp>

class JoystickInterface{
public:
    explicit JoystickInterface(rclcpp::Node::SharedPtr node);
    ~JoystickInterface() = default;

    bool getButtonState(const std::string& button_name);
    double getAxisValue(const std::string& axis_name);

private:
    void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    std::vector<int> buttons_;
    std::vector<double> axes_;
    std::map<std::string, int> button_mapping_;
    std::map<std::string, int> axis_mapping_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
    rclcpp::Node::SharedPtr node_;

    void initMapping();

};