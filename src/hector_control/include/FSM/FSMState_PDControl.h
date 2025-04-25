#ifndef PDCONTROL_H
#define PDCONTROL_H

#include "FSMState.h"

class FSMState_PDControl: public FSMState
{
    public:
        FSMState_PDControl(ControlFSMData *data);
        ~FSMState_PDControl(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();

    private:
        double _legtargetPos[10] = {0.0,  0.1, 0.785, -1.578, 0.785,
                                    0.0, -0.1, 0.785, -1.578, 0.785};

        // double _legtargetPos[10] = {0.0, 0.0, 0.0733698, -2.53195, 0.714347,
        //                             0.0, 0.0, 0.0733698, -2.53195, 0.714347};

        double _armtargetPos[8] = {0.0, 0.35, 0.0, -1.1,
                                   0.0, 0.35, 0.0, -1.1}; // Nominal Pose

        double _legstartPos[10];
        double _armstartPos[8];
        double _duration = 1000;
        double _percent = 0;
        bool isFirstRun = true;
        std::ofstream pose;
};

#endif