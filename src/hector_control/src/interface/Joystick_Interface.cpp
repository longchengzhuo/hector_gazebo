#include "../../include/interface/Joystick_Interface.h"

JoystickInterface::JoystickInterface(rclcpp::Node::SharedPtr node) : node_(node) {  
    
    initMapping();        
    joystick_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickInterface::joystickCallback, this, std::placeholders::_1));

    buttons_.resize(12); 
    axes_.resize(8);    
}

void JoystickInterface::initMapping() {
    // Mapping
    button_mapping_["X"] = 0;
    button_mapping_["A"] = 1;
    button_mapping_["B"] = 2;
    button_mapping_["Y"] = 3;
    button_mapping_["LeftBumper"] = 4;
    button_mapping_["RightBumper"] = 5;
    button_mapping_["LeftTrigger"] = 6;
    button_mapping_["RightTrigger"] = 7;
    button_mapping_["Back"] = 8;
    button_mapping_["Start"] = 9;
    button_mapping_["LeftStickClick"] = 10;
    button_mapping_["RightStickClick"] = 11;

    //axis mapping
    axis_mapping_["CrossKeyX"] = 4;
    axis_mapping_["CrossKeyY"] = 5;
    axis_mapping_["RightStickX"] = 3;
    axis_mapping_["RightStickY"] = 2;
    axis_mapping_["LeftStickX"] = 1;
    axis_mapping_["LeftStickY"] = 0;
}


void JoystickInterface::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    for (size_t i = 0; i < msg->buttons.size(); i++) {
        buttons_[i] = msg->buttons[i];
    }
    for (size_t i = 0; i < msg->axes.size(); i++) {
        axes_[i] = msg->axes[i];
    }
}

bool JoystickInterface::getButtonState(const std::string& button_name) {
    if (button_mapping_.find(button_name) == button_mapping_.end()) {
        std::cerr << "Button name '" << button_name << "' not found.\n";
        return false;
    }    
    int index = button_mapping_[button_name];
    return buttons_[index];
}

double JoystickInterface::getAxisValue(const std::string& axis_name) {
    if (axis_mapping_.find(axis_name) == axis_mapping_.end()) {
        std::cerr << "Axis name '" << axis_name << "' not found.\n";
        return 0.0; 
    }
    int index = axis_mapping_[axis_name];
    return axes_[index];
}
