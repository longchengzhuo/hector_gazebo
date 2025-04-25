#include "../../include/FSM/FSM.h"
#include <iostream>

FSM::FSM(ControlFSMData *data)
    :_data(data)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data);
    _stateList.PD_control = new FSMState_PDControl(_data);

    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize()
{
    count = 0;
    
    _currentState = _stateList.passive;
      
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    // logTraj.open("/home/hector/catkin_drift/logTraj.txt");
    // logDesJointTraj.open("/home/hector/catkin_drift/logDesJointTraj.txt");
}

void FSM::run()
{

    if(!checkSafty())
    {
        // _data->_interface->setPassive();
        // _nextStateName = FSMStateName::PASSIVE;
        // implement soft angle constraitns
        std::cout << "SAFETY ERROR: Angle Exceeded" << std::endl;
        abort();

    }

    if(_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition();
        if(_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if(_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        std::cout << "exited " <<  _currentState->_stateNameStr << std::endl;
        _currentState = _nextState;
        std::cout << "transitioning to " << _currentState->_stateNameStr << std::endl;
        _currentState->enter();
        std::cout << "entered" << std::endl;
        _mode = FSMMode::NORMAL;
        _currentState->run();       
    }
    count++;
}

FSMState* FSM::getNextState(FSMStateName stateName)
{
    switch(stateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
        break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        break;
        case FSMStateName::WALKING:
            return _stateList.walking;
        break;
        case FSMStateName::PDCONTROL:
            return _stateList.PD_control;
        default:
            return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
     //Angle Constraints
    if (count > 50000){
        // Hip Constraint
        if ((_data->_legController->data[0].q(0) < Abad_Leg1_Constraint[0]) || 
          (_data->_legController->data[0].q(0) > Abad_Leg1_Constraint[1])) {
            std::cout << "SAFETY ERROR: Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
            abort();
            return false;
          }
        if ((_data->_legController->data[1].q(0) < Abad_Leg2_Constraint[0]) || 
            (_data->_legController->data[1].q(0) > Abad_Leg2_Constraint[1])) {
            std::cout << "SAFETY ERROR: Abad L Angle Exceeded" << _data->_legController->data[1].q(0) << std::endl;
            abort();
            return false;
            }

        // AbAd Constraint
        if ((_data->_legController->data[0].q(1) < Hip_Leg1_Constraint[0]) ||
            (_data->_legController->data[0].q(1) > Hip_Leg1_Constraint[1])) {
            std::cout << "SAFETY ERROR: Hip R Angle Exceeded" << std::endl;
            abort();
            return false;
            }
        if ((_data->_legController->data[1].q(1) < Hip_Leg2_Constraint[0]) ||
            (_data->_legController->data[1].q(1) > Hip_Leg2_Constraint[1])) {
            std::cout << "SAFETY ERROR: Hip L Angle Exceeded" << _data->_legController->data[1].q(1) << std::endl;
            abort();
            return false;
            }

        //Thigh Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(2) < Thigh_Constraint[0]) || 
            (_data->_legController->data[leg].q(2) > Thigh_Constraint[1])) {
                std::cout << "SAFETY ERROR: Thigh Angle Exceeded" << std::endl;
                abort();
                return false;
            }
        }

        //Calf Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(3) < Calf_Constraint[0]) || 
            (_data->_legController->data[leg].q(3) > Calf_Constraint[1])) {
                std::cout << "SAFETY ERROR: Calf Angle Exceeded" << std::endl;
                abort();
                return false;
            }
        }

        //Ankle Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(4) < Ankle_Constraint[0]) || 
            (_data->_legController->data[leg].q(4) > Ankle_Constraint[1])) {
                std::cout << "SAFETY ERROR: Ankle Angle Exceeded" << std::endl;
                abort();
                return false;
            }
        }

        

        return true;
    }else{
        return true;
    }
    
}

// Eigen::VectorXd FSM::getTrajectory(){
  
//   // Roll out MPC trajectory desired structure of trajectory is as follows
//   // mpcTraj.row() = {x,  y,  z,  r,  p,  y,  q[left],  q[right],
//   //                  vx, vy, vz, wx, wy, wz, wq[left], wq[right],
//   //                  contact[left], contact[right]}
//   int totalSize = 98;
//   trajectory.resize(totalSize);
//   trajectory.setZero();
 
//   // Position and Velocity
//   for (int i = 0; i < 3; i++) {
//     trajectory(i) = _data->_stateEstimator->getResult().position(i);
//     trajectory(i + 3) = _data->_stateEstimator->getResult().rpy(i);
//   }
 
// //   // Orientation and Angular Velocity
//   for (int i = 0; i < 3; i++) {
//     trajectory(i + 16) = _data->_stateEstimator->getResult().vWorld(i);;
//     trajectory(i + 19) = _data->_stateEstimator->getResult().omegaWorld(i);
//   }
 
//   // Orientation and Angular Velocity
// //   for (int i = 0; i < 3; i++) {
// //     trajectory(i + 16) = _data->_stateEstimator->getResult().omegaWorld(i);
// //     trajectory(i + 19) = _data->_stateEstimator->getResult().omegaBody(i);
// //   }

//   // Joint Angles and Velocities
//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 5; i++) {
//       trajectory(6 + leg*5 + i) = _data->_lowLevelController->data[leg].q(i);
//       trajectory(22 + leg*5 + i) = _data->_lowLevelController->data[leg].qd(i);
//     }
//   }

//   // Torque cmds and estimates
//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 5; i++) {
//       trajectory(32 + leg*5 + i) = _data->_lowLevelController->data[leg].tau(i);
//       trajectory(42 + leg*5 + i) = _data->_lowState->motorStates[leg*5+i].tauEst;
//     }
//   }

// //   data->_lowLevelController->commands[leg].pDes
//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 3; i++) {
//       trajectory(52 + leg*3 + i) = _data->_lowLevelController->commands[leg].pDes(i);
//     //   std::cout << "pDes FSM** :  \n" << _data->_lowLevelController->commands[leg].pDes(i);
//     }
//     //   std::cout << "pDes# " << leg << " FSM** :  \n " << _data->_lowLevelController->commands[leg].pDes;

//   }


//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 3; i++) {
//       trajectory(58 + leg*3 + i) = _data->_lowLevelController->data[leg].p(i);
//     }
//   }  

//       // data._lowLevelController->commands[foot].feedforwardForce = f_ff[foot];
//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 3; i++) {
//       trajectory(64 + leg*3 + i) = _data->_lowLevelController->commands[leg].feedforwardForce(i);
//     }
//   }  


//   // Contact States //Implement this in the future
//   trajectory(totalSize - 8) = _data->_lowCmd->contactStates[0];
//   trajectory(totalSize - 7) = _data->_lowCmd->contactStates[1];

//   for(int leg = 0; leg < 2; leg++){
//     for (int i = 0; i < 3; i++) {
//       trajectory(76 + leg*3 + i) = _data->_lowLevelController->data[leg].ff(i);
//   }
//   }

//   for(int leg = 0; leg < 2; leg++){
//     for (int i = 0; i < 3; i++) {
//       trajectory(82 + leg*3 + i) = _data->_lowLevelController->data[leg].v(i);     
//   }
//   }

//   trajectory(totalSize - 10) = _data->_lowCmd->MPC_contact[0];
//   trajectory(totalSize - 9) = _data->_lowCmd->MPC_contact[1];

//   for(int i = 0; i < 3; i++){
//     trajectory(totalSize- (6-i) ) = _data->_lowState->imus[0].accelerometer[i];
//   }
//   for(int i = 0; i < 3; i++){
//     trajectory(totalSize - (3-i)) = _data->_lowState->imus[0].gyroscope[i];
//   }
//   return trajectory;
// }
 
// Eigen::VectorXd FSM::getDesiredJointAngles(){
//   desiredJointAngles.resize(10);
//   desiredJointAngles.setZero();
//   for (int leg = 0; leg < 2; leg++) {
//     for (int i = 0; i < 5; i++) {
//       desiredJointAngles(leg*5 + i) = _data->_lowLevelController->commands[leg].qDes(i);
//     }
//   }
//   //add offset to the desired joint angles
//     // desiredJointAngles(2) += 0.3*M_PI;
//     // desiredJointAngles(3) -= 0.6*M_PI;
//     // desiredJointAngles(4) += 0.3*M_PI;
 
//     // desiredJointAngles(7) += 0.3*M_PI;
//     // desiredJointAngles(8) -= 0.6*M_PI;
//     // desiredJointAngles(9) += 0.3*M_PI;
//   return desiredJointAngles;
// }
 
// void FSM::logData()
// {
//     // #ifdef LOG_TRAJECTORY
//         logTraj << getTrajectory().transpose() << std::endl;
//         logDesJointTraj << getDesiredJointAngles().transpose() << std::endl;
//     // #endif
// }