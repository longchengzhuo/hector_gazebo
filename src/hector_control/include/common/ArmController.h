#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Biped.h"

// Data sent from control algorithm to arms
struct ArmControllerCommand{
    ArmControllerCommand() {zero();}

    void zero();

    Vec4<double> qDes, qdDes, tau;
    Vec3<double> pDes, vDes;
    Mat4<double> kpJoint, kdJoint;
    Vec6<double> feedforwardForce;
    Mat3<double> kpCartesian;
    Mat3<double> kdCartesian;
};

struct ArmControllerData{
    ArmControllerData() {zero();}
    void setBiped(Biped& biped) {hector = &biped;}

    void zero();
    Vec4<double> q, qd;
    Vec3<double> p, v;
    Mat64<double> J;
    Mat34<double> J2;
    Vec4<double> tau;
    Biped* hector;
};

class ArmController{
    public:
        ArmController(Biped& biped) : _biped(biped){
            for (auto& dat : data) dat.setBiped(_biped);
            for (int i = 0; i<2; i++){
                commands[i].zero();
                data[i].zero();
            }
        };

        void zeroCommand();
        void edampCommand(double gain);
        void updateData(const LowlevelState* state);
        void updateCommand(LowlevelCmd* cmd);
        bool __simulated_robot;

        ArmControllerCommand commands[2];
        ArmControllerData data[2];
        Biped& _biped;

        float arm_gear_ratio = 1.417;
};

#endif