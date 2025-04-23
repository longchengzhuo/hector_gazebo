// Author: 
// Date: 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HECTOR_MOTOR_CONTROLLER_H
#define HECTOR_MOTOR_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace hector_motor_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn; 

class HectorMotorController : public controller_interface::ControllerInterface {

    public:
        HectorMotorController();

        CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // controller_interface::return_type configure() override;        

//        controller_interface::return_type update() override;

    protected:

        std::vector<std::string> joint_names_;
        std::string interface_name_;

        // Command subscribers and Controller State publisher
        using ControllerCommandMsg = control_msgs::msg::JointJog;

        rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ = nullptr;
        realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_command_;

        using ControllerStateMsg = control_msgs::msg::JointControllerState;
        using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

        rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
        std::unique_ptr<ControllerStatePublisher> state_publisher_;

}; // class hectorMotorController

} // namespace hector_motor_controller

#endif // HECTOR_MOTOR_CONTROLLER_H
