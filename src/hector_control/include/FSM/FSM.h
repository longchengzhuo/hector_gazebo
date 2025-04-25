#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Passive.h"
#include "FSMState_Walking.h"
#include "FSMState_PDControl.h"
#include "../common/enumClass.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_Passive *passive;
    FSMState_Walking *walking;
    FSMState_PDControl *PD_control;

   
    void deletePtr(){
        delete invalid;
        delete passive;
        delete walking;
        delete PD_control;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();
    private:
        FSMState* getNextState(FSMStateName stateName);
        // FSMStateName checkTransition() override;
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;
        Eigen::VectorXd getTrajectory();
        Eigen::VectorXd getDesiredJointAngles();
        void logData();
        Eigen::VectorXd trajectory;
        Eigen::VectorXd desiredJointAngles;

        ofstream logTraj;
        ofstream logDesJointTraj;

        //Angle safety constraint
        double Abad_Leg1_Constraint[2] = {-60 * (3.1415/180), 60 * (3.1415/180)};
        double Abad_Leg2_Constraint[2] = {-45 * (3.1415/180), 45 * (3.1415/180)};
        double Hip_Leg1_Constraint[2] = {-45* (3.1415/180.0), 45 * (3.1415/180)};
        double Hip_Leg2_Constraint[2] = {-45 * (3.1415/180), 45 * (3.1415/180)};
        double Thigh_Constraint[2] = {0 * (3.1415/180), 120 * (3.1415/180)};
        double Calf_Constraint[2] = {-150 * (3.1415/180), -10 * (3.1415/180)};
        double Ankle_Constraint[2] = {-10 * (3.1415/180), 95 * (3.1415/180)};        
};

#endif