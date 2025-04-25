#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"

class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}

        FSMStateName _stateName;
        std::string _stateNameStr;

    protected:

        ControlFSMData *_data;
        FSMStateName _nextStateName;
        int motionTime = 0;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;
        std::ofstream raw_angle;

        std::ofstream com_pos;
        std::ofstream angle;
        std::ofstream torque;
        std::ofstream footposition;
        std::ofstream rpy_input;
        std::ofstream force;
        std::ofstream omega;
        std::ofstream acceleration;
        std::ofstream tau_est;
        std::ofstream temperature;
        std::ofstream corrected_angle;
        std::ofstream T265_qua;
        std::ofstream motor_error;   

        bool flagGaitTimer_Stand;
        bool flagGaitTimer_Walk;

        bool flagTempTimer_Stand;
        bool flagTempTimer_Walk;

        int waveTimer;
        int hiFiveTimer;
        int pickupTimer;
        int pickupSeq;
        int hiFiveSeq;     

        // double mpcdt = 55;

        bool buttonA;
        bool buttonB;
        bool buttonX;
        bool buttonY;
        bool buttonRB;
        bool left_trigger;
        bool left_shoulder;
        bool right_trigger;
        bool right_shoulder;
        bool buttonStart;
        bool buttonback;
        bool flagWalk;
        bool flagZeroVel;
        bool flagHi;
        double motionTimeHi;

        float xAxis;
        float yAxis;
        float zAxis;
        float vx_command;
        float vy_command;

        double motionTimeRecovery;

        //reference arm traj:
        double qdes_arm[8] = {0.1, 0.2, 0.15, -0.6, -0.1, 0.2, -0.15, -0.6};
        double qdes_nominal[8] = {0.1, 0.2, 0.2, -0.6, -0.1, 0.2, -0.2, -0.6};
        double qdes_up[8] = { 0.1, 0.2, 0.25, -0.6, 0.1, -1.5, -1.5, -1.5};

        //dab
        double qdes_dab_l[8] = {-0.1, -0.1, 1.4, -3,  0.2, 0.1, -1.7, 0 };

        //HiFive
        double qdes_hiFiveUp[8] = { 0.1, 0.2, 0.25, -0.6,  -0.1, -2.0, 0.0, -1.5};
        double qdes_hiFive[8] = { 0.1, 0.2, 0.25, -0.6,  -0.1, -2.0, 0.0, -1.0};

        //moveboxes:
        double qdes_prepare[8] = {0.0, -0.785, 0.20, -0.785, 0.0, -0.785, -0.20, -0.785};
        double qdes_clamp[8] = {0.0, -0.785, -0.10, -0.785, 0.0, -0.785, 0.10, -0.785};
        double qdes_hold[8] = {0.0, 0.0, -0.10, -1.57, 0.0, 0.0, 0.10, -1.57};

};

#endif // FSMSTATE_H