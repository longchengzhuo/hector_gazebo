
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/ROSIO.h"
#include "../include/interface/Interface.h"
#include "../include/FSM/FSM.h"
#include "../include/interface/Joystick_Interface.h"


bool running = true;

void ShutDown(int sig)
{
    std::cout << "Stopping ROS2 node..." << std::endl;
    running = false;
    rclcpp::shutdown(); // Ensure ROS nodes are shutdown
    exit(0); // Ensure process exits
}

// void setProcessScheduler(){
//     struct sched_param schedParam;
//     schedParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
//     if (sched_setscheduler(0, SCHED_FIFO, &schedParam) != 0)
//     {
//         std::cout << "Failed to set process scheduler" << std::endl;
//         abort();
//     }
// }
void setThreadScheduler(pthread_t thread, int priority)
{
    struct sched_param schedParam;
    schedParam.sched_priority = priority;
    if (pthread_setschedparam(thread, SCHED_FIFO, &schedParam) != 0)
    {
        std::cerr << "Failed to set real-time priority: " << strerror(errno) << std::endl;
    }
}


void runFSMController(FSM* _FSMController)
{
    pthread_t this_thread = pthread_self();
    setThreadScheduler(this_thread, 99);
    bool first_run = true;
    rclcpp::Rate rate(1000); 
    while (running && rclcpp::ok())
    {
        auto start_time = std::chrono::high_resolution_clock::now(); 
        _FSMController->run();
        auto end_time = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        int remaining_time = 950 - static_cast<int>(duration.count());
        if (first_run){
            rate.sleep();
            first_run = false;
        }
        
        if (remaining_time > 0){
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_time));
        }
    }
}

void runRecv(ROSIO* _ioROS, LowlevelState* _state, LegController* _legController, ArmController* _armController)
{
    pthread_t this_thread = pthread_self();
    setThreadScheduler(this_thread, 99);
    bool first_run = true;
    rclcpp::Rate rate(1000); 
    while (running && rclcpp::ok())
    {
        auto start_time = std::chrono::high_resolution_clock::now(); 
        _ioROS->recvState(_state);
        // _legController->updateData(_state);
        // _armController->updateData(_state);
        auto end_time = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        int remaining_time = 950 - static_cast<int>(duration.count());
        if (first_run){
            rate.sleep();
            first_run = false;
        }


        if (remaining_time > 0){
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_time));
        }
    }
}

void runSend(ROSIO* _ioROS,  LowlevelCmd* _cmd, LegController* _legController, ArmController* _armController)
{
    pthread_t this_thread = pthread_self();
    setThreadScheduler(this_thread, 99);
    bool first_run = true;
    rclcpp::Rate rate(1000); 
    while (running && rclcpp::ok())
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        // _legController->updateCommand(_cmd, 3000);
        // _armController->updateCommand(_cmd);
        _ioROS->sendCmd(_cmd);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        int remaining_time = 950 - static_cast<int>(duration.count());
        if (first_run){
            rate.sleep();
            first_run = false;
        }


        if (remaining_time > 0){
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_time));
        }
        auto end_time2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end_time2 - start_time);
        // std::cout << "send time " << duration2.count() << std::endl;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, ShutDown);
    
    auto node = std::make_shared<rclcpp::Node>("hector_control");

    // setProcessScheduler();  

    bool simulated_robot = false;
    bool physical_robot = false;

    node->declare_parameter<bool>("simulated_robot", false);
    node->declare_parameter<bool>("physical_robot", false);

    // Get parameters from launch file
    node->get_parameter("simulated_robot", simulated_robot);
    node->get_parameter("physical_robot", physical_robot);

    std::cout << "Simulated Robot" << simulated_robot << std::endl;
    std::cout << "Physical Robot" << physical_robot << std::endl;

    std::cout << "starting setup " << std::endl;
    double dt = 0.001;
    Biped biped;
    LegController* legController = new LegController(biped);
    ArmController* armController = new ArmController(biped);
    LowlevelCmd* lowCmd = new LowlevelCmd();
    LowlevelState* lowState = new LowlevelState();
    auto ioROS = std::make_shared<ROSIO>(node);
    auto joystickInterface = std::make_shared<JoystickInterface>(node);
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(lowState,
                                                                          legController->data,
                                                                          &stateEstimate);

        stateEstimator->addEstimator<CheaterOrientationEstimator>();
        stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
        // stateEstimator->addEstimator<OrientationEstimator>();
        // stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_legController = legController;
    _controlData->_armController = armController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_lowCmd = lowCmd;
    _controlData->_lowState = lowState;
    _controlData->_joystickInterface = joystickInterface.get();
    _controlData->_interface = ioROS.get();
    _controlData->_stateEstimator = stateEstimator;
    


    if (simulated_robot)
    {
        // _controlData->_interface = ioInter;
        _controlData->_legController->__simulated_robot = true;
        _controlData->_armController->__simulated_robot = true;
        std::cout << "\n gazebo sim is running \n" << std::endl;
    }
    else if (physical_robot)
    {
        // _controlData->_interface = iosdk;
        _controlData->_legController->__simulated_robot = false;
        _controlData->_armController->__simulated_robot = false;
        //  stateEstimator->addEstimator<T265TrackingCameraEstimator>();   
        std::cout << "\n gazebo sim is NOT running \n" << std::endl;
    }
    else
    {
        std::cout << "\n no interface is running \n" << std::endl;
        abort();
    }
    FSM* _FSMController = new FSM(_controlData);
    std::cout << "setup complete" << std::endl;

    std::cout << "Starting Threads..." << std::endl;

    // FSM in a separate thread
    std::thread fsm_thread(runFSMController, _FSMController);
    std::thread recv_thread(runRecv, ioROS.get(), lowState, legController, armController);
    std::thread send_thread(runSend, ioROS.get(), lowCmd, legController, armController);

    rclcpp::spin(node);

    std::cout << "Shutting down threads..." << std::endl;

    // Join threads before exiting
    fsm_thread.join();
    recv_thread.join();
    send_thread.join();

    std::cout << "All threads stopped." << std::endl;

    delete _controlData;
    rclcpp::shutdown();
    return 0;
}
