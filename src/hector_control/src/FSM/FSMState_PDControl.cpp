#include <iostream>
#include "../../include/FSM/FSMState_PDControl.h"

FSMState_PDControl::FSMState_PDControl(ControlFSMData *data)
            :FSMState(data, FSMStateName::PDCONTROL, "PDControl"){

            }

void FSMState_PDControl::enter()
{
    
    std::cout << "entered PDCONTROL" << std::endl;
    _data->_legController->updateData(_data->_lowState);
    _data->_armController->updateData(_data->_lowState);


    _data->_legController->zeroCommand();
    _data->_armController->zeroCommand();


    for (int leg = 0; leg < 2; leg++)
    {
        if (!_data->_legController->__simulated_robot)
        {
            _data->_legController->commands[leg].kpJoint << 30, 0, 0, 0, 0,
                                                      0, 30, 0, 0, 0,
                                                      0, 0, 30, 0, 0,
                                                      0, 0, 0, 30, 0,
                                                      0, 0, 0, 0, 30;

            _data->_legController->commands[leg].kdJoint << 1, 0, 0, 0, 0,
                                                        0, 1, 0, 0, 0,
                                                        0, 0, 1, 0, 0,
                                                        0, 0, 0, 1, 0,
                                                        0, 0, 0, 0, 1;
        }
        else
        {
            _data->_legController->commands[leg].kpJoint << 30, 0, 0, 0, 0,
                                                      0, 30, 0, 0, 0,
                                                      0, 0, 30, 0, 0,
                                                      0, 0, 0, 30, 0,
                                                      0, 0, 0, 0, 1;

            _data->_legController->commands[leg].kdJoint << 1, 0, 0, 0, 0,
                                                        0, 1, 0, 0, 0,
                                                        0, 0, 1, 0, 0,
                                                        0, 0, 0, 1, 0,
                                                        0, 0, 0, 0, 0.1;
        }
        
        // _data->_legController->commands[leg].kpJoint << 5, 0, 0, 0, 0,
        //                                               0, 5, 0, 0, 0,
        //                                               0, 0, 5, 0, 0,
        //                                               0, 0, 0, 5, 0,
        //                                               0, 0, 0, 0, 5;

        // _data->_legController->commands[leg].kdJoint << 1, 0, 0, 0, 0,
        //                                               0, 1, 0, 0, 0,
        //                                               0, 0, 1, 0, 0,
        //                                               0, 0, 0, 1, 0,
        //                                               0, 0, 0, 0, 1;

        for (int j = 0; j < 5; j++)
        {
            _legstartPos[leg*5+j] = _data->_legController->data[leg].q(j);
        }
    }


    for (int arm = 0; arm < 2; arm++){
        _data->_armController->commands[arm].kpJoint << 5, 0, 0, 0,
                                                      0, 5, 0, 0,
                                                      0, 0, 5, 0,
                                                      0, 0, 0, 5;

        _data->_armController->commands[arm].kdJoint << 1, 0, 0, 0,
                                                      0, 1, 0, 0,
                                                      0, 0, 1, 0,
                                                      0, 0, 0, 1;

        for (int j = 0; j < 4; j++)
        {
            _armstartPos[arm*4+j] = _data->_armController->data[arm].q(j);
        }
    }
    motionTime = 0;
    isFirstRun = true;

}

void FSMState_PDControl::run()
{
    motionTime++;
    std::cout << "Current state is PD state" << std::endl;

    _data->_legController->updateData(_data->_lowState);
    _data->_armController->updateData(_data->_lowState);

    _data->_stateEstimator->run();

    

    _percent += 1.0 / _duration;
    _percent = _percent > 1 ? 1 : _percent;
    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < 5; j++)
        {
            _data->_legController->commands[leg].qDes(j) = (1 - _percent) * _legstartPos[leg*5+j]+_percent*_legtargetPos[leg*5+j];
        }   
    }

    for (int arm = 0; arm < 2; arm++)
    {
        for (int j = 0; j < 4; j++)
         {
             _data->_armController->commands[arm].qDes(j) = (1 - _percent)*_armstartPos[arm*4+j]+_percent*_armtargetPos[arm*4+j];
         }
    }

    _data->_legController->updateCommand(_data->_lowCmd, 3000);
    _data->_armController->updateCommand(_data->_lowCmd);

     
}

void FSMState_PDControl::exit()
{
    _percent = 0;
    // _data->_interface->cmdPanel->setCmdNone(); //comment out for now
}

FSMStateName FSMState_PDControl::checkTransition()
{
    if(_data->_joystickInterface->getButtonState("Y")){
        std::cout << "transition from PD to Passive" << std::endl;
        return FSMStateName::WALKING;   
    }  else if(_data->_joystickInterface->getButtonState("LeftBumper") || _data->_joystickInterface->getButtonState("RightBumper")){
            std::cout << "Kill button pressed" << std::endl;
            abort();
            return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::PDCONTROL;
    }
}