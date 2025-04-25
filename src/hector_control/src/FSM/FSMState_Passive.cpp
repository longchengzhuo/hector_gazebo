#include "../../include/FSM/FSMState_Passive.h"

FSMState_Passive::FSMState_Passive(ControlFSMData *data):
                  FSMState(data, FSMStateName::PASSIVE, "passive"){}

void FSMState_Passive::enter()
{
    _data->_legController->zeroCommand();
    _data->_armController->zeroCommand();
    for(int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 5; j++){
            _data->_legController->commands[i].kdJoint(j,j) = 0.2;
        }

        for (int j = 0; j < 4; j++){
            _data->_armController->commands[i].kdJoint(j,j) = 0.2;
        }
    }
    for (int leg = 0; leg < 2; leg++){
        std::cout << "Leg " << leg << " qDes: ";
        for (int i = 0; i < 5; i++){
            std::cout << _data->_legController->commands[leg].qDes(i) << ",  ";
            // std::cout << _data->_legController->data[leg].q(i) << ",  ";
        }
    }    
    // abort();

}

void FSMState_Passive::run()
{
    std::cout << "Current state is passive state" << std::endl;
    _data->_armController->updateData(_data->_lowState);
    _data->_legController->updateData(_data->_lowState);

    // _data->_armController->updateCommand(_data->_lowCmd);
    // _data->_legController->updateCommand(_data->_lowCmd, 5000);
    // std::cout << "Motor Angle is: ";
    // for (int leg = 0; leg < 2; leg++){
    //  for (int i = 0; i < 5; i++){
    //         std::cout << _data->_lowLevelController->data[leg].q(i) << ",  ";
    //     }
    // }
    // std::cout << "\n";
    // for (int arm = 0; arm < 2; arm++){
    //  for (int j = 0; j < 4; j++){
    // std::cout << "arm " <<  arm << "q " << j << ": "<<_data->_lowLevelController->armData[arm].q(j) << "\n";}
    // }

    //print all stateEstimator data
    // std::cout << "StateEstimator Data: " << std::endl;
    // std::cout << "Position: " << _data->_stateEstimator->getResult().position.transpose() << std::endl;
    // std::cout << "Velocity: " << _data->_stateEstimator->getResult().vWorld.transpose() << std::endl;
    // std::cout << "RPY: " << _data->_stateEstimator->getResult().rpy.transpose() << std::endl;
    // std::cout << "OmegaWorld: " << _data->_stateEstimator->getResult().omegaWorld.transpose() << std::endl;
    // std::cout << "OmegaBody: " << _data->_stateEstimator->getResult().omegaBody.transpose() << std::endl;    
    _data->_legController->updateCommand(_data->_lowCmd, 3000);
    _data->_armController->updateCommand(_data->_lowCmd);

}

void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }
}

FSMStateName FSMState_Passive::checkTransition()
{
    //TODO: Implement transition logic
    if(_data->_joystickInterface->getButtonState("Start"))
    {   std::cout << "Transitioning to PD Control" << std::endl;
        return FSMStateName::PDCONTROL;
    }
    if(_data->_joystickInterface->getButtonState("LeftBumper") || _data->_joystickInterface->getButtonState("RightBumper")){
            std::cout << "Kill button pressed" << std::endl;
            abort();
            return FSMStateName::PASSIVE;

    }
    return FSMStateName::PASSIVE;
}