#ifndef ROSIO_H
#define ROSIO_H

#include "rclcpp/rclcpp.hpp"
#include<boost/array.hpp>
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "Interface.h"
#include <string>
#include <thread>

#include "laser_interfaces/msg/robot_state.hpp"
#include "laser_interfaces/msg/robot_command.hpp"
#include "laser_interfaces/msg/estimated_state.hpp"

class ROSIO: public Interface
{
    public:
        explicit ROSIO(rclcpp::Node::SharedPtr node);
        ~ROSIO();

        void sendCmd(const LowlevelCmd *cmd);
        void recvState(LowlevelState *state);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<laser_interfaces::msg::RobotState>::SharedPtr _hector_sub;
        rclcpp::Subscription<laser_interfaces::msg::EstimatedState>::SharedPtr _estimator_sub;
        rclcpp::Publisher<laser_interfaces::msg::RobotCommand>::SharedPtr _hector_pub;

        laser_interfaces::msg::RobotCommand _robotCommand;

        LowlevelState _intermediate_state;


        void StateCallback(const laser_interfaces::msg::RobotState::SharedPtr msg);
        void EstimatorCallback(const laser_interfaces::msg::EstimatedState::SharedPtr msg);
};   

#endif