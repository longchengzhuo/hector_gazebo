#include "../../include/common/ArmController.h"

#include <iostream>

void ArmControllerCommand::zero(){
    tau = Vec4<double>::Zero();
    qDes = Vec4<double>::Zero();
    qdDes = Vec4<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    kpCartesian = Mat3<double>::Zero();
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat4<double>::Zero();
    kdJoint = Mat4<double>::Zero();
}

// Zero Arm Data
void ArmControllerData::zero(){
    q = Vec4<double>::Zero();
    qd = Vec4<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J = Mat64<double>::Zero();
    J2 = Mat34<double>::Zero();
    tau = Vec4<double>::Zero();
}

void ArmController::zeroCommand(){
    for (int i = 0; i<2; i++){
        commands[i].zero();
    }
}

void ArmController::updateData(const LowlevelState* state){
    // Grab data and save into armcontroller data
    for (int arm = 0; arm < 2; arm++){
        for(int j=0; j<4;j++){
            data[arm].q(j) = state->motorState[10+j+arm*4].q;
            data[arm].qd(j)= state->motorState[10+j+arm*4].dq;
            data[arm].tau(j)=state->motorState[10+j+arm*4].tauEst;
        }

        // Implement Gear Ratio on Elbow
        if (!__simulated_robot){
            data[arm].q(3) = data[arm].q(3) / arm_gear_ratio;
            data[arm].qd(3) = data[arm].qd(3) / arm_gear_ratio;
            data[arm].tau(3) = data[arm].tau(3) * arm_gear_ratio;
        }


        // std::cout << "In Arm Controller State " << arm << std::endl;
        //     for (int i = 0; i < 4; i++){
        //         std::cout << data[arm].q(i) << " ";
        //     }
        // std::cout<<std::endl;

        //v and p calculation using jacobian, add later
    }
}

void ArmController::updateCommand(LowlevelCmd* cmd){
    for (int arm = 0; arm < 2; arm++){
            // Implement Gear Ratio
            if (!__simulated_robot){
            commands[arm].qDes(3) = commands[arm].qDes(3) * arm_gear_ratio;
            commands[arm].qdDes(3) = commands[arm].qdDes(3) * arm_gear_ratio;
            commands[arm].tau(3) = commands[arm].tau(3) / arm_gear_ratio;
            }
            //arm jacobian calculation goes here
            for (int j = 0; j < 4; j++){
                cmd->motorCmd[10+j+arm*4].tau = commands[arm].tau(j);
                cmd->motorCmd[10+j+arm*4].q = commands[arm].qDes(j);
                cmd->motorCmd[10+j+arm*4].dq = commands[arm].qdDes(j);
                cmd->motorCmd[10+j+arm*4].Kp = commands[arm].kpJoint(j,j);
                cmd->motorCmd[10+j+arm*4].Kd = commands[arm].kdJoint(j,j);
            }
    }
}

