#ifndef WALKING_H
#define WALKING_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
#include <chrono>

class FSMState_Walking: public FSMState
{
    public:
        FSMState_Walking(ControlFSMData *data);
        ~FSMState_Walking(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
        std::chrono::high_resolution_clock::time_point start_time;;
    
    private:
        ConvexMPCLocomotion Cmpc;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
        int gait_num = 1;
        int motiontime = 0;
        double _armtargetPos[8] = {0.0, 0.35, 0.0, -1.1,
                                    0.0, 0.35, 0.0, -1.1}; // Nominal Pose
        double _armstartPos[8];
        double _duration = 1000;
        double _percent = 0;
};

#endif