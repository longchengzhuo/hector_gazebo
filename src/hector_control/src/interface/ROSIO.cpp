#include "../../include/interface/ROSIO.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <memory>

inline void RosShutDown(int sig){
    RCLCPP_INFO(rclcpp::get_logger("ROSIO"), "ROS interface shutting down!");
    rclcpp::shutdown();
}

ROSIO::ROSIO(rclcpp::Node::SharedPtr node) : node_(node)
{
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;

    _hector_pub = node_->create_publisher<laser_interfaces::msg::RobotCommand>("/Hector_Command", 10);

    _hector_sub = node_->create_subscription<laser_interfaces::msg::RobotState>("/Hector_State", 1, std::bind(&ROSIO::StateCallback, this, std::placeholders::_1));
    _estimator_sub = node_->create_subscription<laser_interfaces::msg::EstimatedState>("/estimated_state", 1, std::bind(&ROSIO::EstimatorCallback, this, std::placeholders::_1));

    signal(SIGINT, RosShutDown);

}

ROSIO::~ROSIO()
{
    rclcpp::shutdown();
}

void ROSIO::sendCmd(const LowlevelCmd *cmd)
{
    for(int i = 0; i < 18; i++){     
        _robotCommand.motor_command[i].q = cmd->motorCmd[i].q;
        _robotCommand.motor_command[i].dq = cmd->motorCmd[i].dq;
        _robotCommand.motor_command[i].tau = cmd->motorCmd[i].tau;
        _robotCommand.motor_command[i].kd = cmd->motorCmd[i].Kd;
        _robotCommand.motor_command[i].kp = cmd->motorCmd[i].Kp;
    }
    for (int i = 0; i < 2; i++)
    {
        _robotCommand.mpc_phase[i] = cmd->MPC_phase[i];
    }
    _hector_pub->publish(_robotCommand);
    // std::cout << "Command sent" << std::endl;
}

void ROSIO::recvState(LowlevelState *state)
{
    for(int i = 0; i < 18; i++)
    {
        state->motorState[i].q = _intermediate_state.motorState[i].q;
        state->motorState[i].dq = _intermediate_state.motorState[i].dq;
        state->motorState[i].tauEst = _intermediate_state.motorState[i].tauEst;
    }
    for(int i = 0; i < 3; i++){
        state->position[i] = _intermediate_state.position[i];
        state->vWorld[i] = _intermediate_state.vWorld[i];
        state->imu.gyroscope[i] = _intermediate_state.imu.gyroscope[i];
        state->imu.accelerometer[i] = _intermediate_state.imu.accelerometer[i];
        state->rpy[i] = _intermediate_state.rpy[i];
    }
    // for (int i = 0; i < 4; i++)
    // {
    //     state->imu.quaternion[i] = _intermediate_state.imu.quaternion[i];
    // }
    // std::cout << "Position is" << state->position[0] << " " << state->position[1] << " " << state->position[2] << std::endl;
    // std::cout << "Velocity is" << state->vWorld[0] << " " << state->vWorld[1] << " " << state->vWorld[2] << std::endl;
    // std::cout << "RPY is" << state->rpy[0] << " " << state->rpy[1] << " " << state->rpy[2] << std::endl;
    // std::cout << "Gyroscope is" << state->imu.gyroscope[0] << " " << state->imu.gyroscope[1] << " " << state->imu.gyroscope[2] << std::endl;
    // std::cout << "Accelerometer is" << state->imu.accelerometer[0] << " " << state->imu.accelerometer[1] << " " << state->imu.accelerometer[2] << std::endl;
    // std::cout << "Quaternion is" << state->imu.quaternion[0] << " " << state->imu.quaternion[1] << " " << state->imu.quaternion[2] << " " << state->imu.quaternion[3] << std::endl;
}

void ROSIO::StateCallback(const laser_interfaces::msg::RobotState::SharedPtr msg)
{
    for(int i = 0; i < 18; i++)
    {
        _intermediate_state.motorState[i].q = msg->motor_state[i].q;
        _intermediate_state.motorState[i].dq = msg->motor_state[i].dq;
        _intermediate_state.motorState[i].tauEst = msg->motor_state[i].tauest;
    }

    for(int i = 0; i < 3; i++)
    {
        // Uncomment following if you want to use simulator state
        _intermediate_state.position[i] = msg->body_position[i];
        _intermediate_state.vWorld[i] = msg->body_velocity[i];

        _intermediate_state.imu.gyroscope[i] = msg->imu[0].gyroscope[i];
        _intermediate_state.imu.accelerometer[i] = msg->imu[0].accelerometer[i];
    }
    // for (int i = 0; i < 4; i++)
    // {
    //     _intermediate_state.imu.quaternion[i] = msg->imu[0].quaternion[i];
    // }
}

void ROSIO::EstimatorCallback(const laser_interfaces::msg::EstimatedState::SharedPtr msg)
{
    for(int i = 0; i < 3; i++)
    {
        // Uncomment following if you want to use estimator state
        // _intermediate_state.position[i] = msg->position[i];
        // _intermediate_state.vWorld[i] = msg->v_world[i];
        _intermediate_state.rpy[i] = msg->rpy[i];
    }
    // for (int i = 0; i < 4; i++)
    // {
    //     _intermediate_state.imu.quaternion[i] = msg->imu[0].quaternion[i];
    // }
}