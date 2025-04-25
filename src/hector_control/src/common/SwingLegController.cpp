#include "../../include/common/SwingLegController.h"

/******************************************************************************************************/
/******************************************************************************************************/

swingLegController::swingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    L_hipYawLocation = data->_biped->getHipYawLocation(0);
    L_hipRollLocation = data->_biped->getHipRollLocation(0);
    R_hipYawLocation = data->_biped->getHipYawLocation(1);
    R_hipRollLocation = data->_biped->getHipRollLocation(1);
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.09);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }

}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::initSwingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    L_hipYawLocation = data->_biped->getHipYawLocation(0);
    L_hipRollLocation = data->_biped->getHipRollLocation(0);
    R_hipYawLocation = data->_biped->getHipYawLocation(1);
    R_hipRollLocation = data->_biped->getHipRollLocation(1);
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.09);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingLeg(){
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
    updateSwingStates();
    updateSwingTimes();
    computeFootPlacement();     
    computeFootDesiredPosition();
    setDesiredJointState();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){    
    pFoot_w[i] =  seResult.position + seResult.rBody.transpose() 
                * ( data->_biped->getHipYawLocation(i) + data->_legController->data[i].p); 
    }

}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingStates(){
    swingStates = gait->getSwingSubPhase();
    contactStates = gait->getContactSubPhase();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingTimes(){
    for(int leg = 0; leg < nLegs; leg++){
        if(firstSwing[leg]){
            swingTimes[leg] = _dtSwing * gait->_swing;
        }else{
            swingTimes[leg] -= _dt;
            if(contactStates[leg] > 0){
                firstSwing[leg] = true;
            }            
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootPlacement(){

    auto &stateCommand = data->_desiredStateCommand;

    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;

    v_des_world = seResult.rBody.transpose() * v_des_robot;    
    for(int foot = 0; foot < nLegs; foot++){
        footSwingTrajectory[foot].setHeight(0.09);
        Vec3<double> Pf = seResult.position +
                seResult.rBody.transpose() * (data->_biped->getHipYawLocation(foot))
                + seResult.vWorld * swingTimes[foot];

        double p_rel_max =  0.3;
        double pfx_rel   =  1.0 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                            0.25  * (seResult.vWorld[0] - v_des_world[0]);

        double pfy_rel   =  1.0 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                            0.25  * (seResult.vWorld[1] - v_des_world[1]);
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel; 
        Pf[2] = 0.0;

        // std::cout << "Foot " << foot << " Pfinal: " << Pf.transpose() << "\n";  
        // std::cout << "seResult.vWorld[1]: " << seResult.vWorld[1] << "\n";


        footSwingTrajectory[foot].setFinalPosition(Pf);        

    }
}


/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] > 0){
            if (firstSwing[foot]){
            //   std::cout << "firstSwing[" << foot << "] = "<< firstSwing[foot] << "\n"; 
                // std::cout << "\n\nFirst Swing\n\n"; 
              firstSwing[foot] = false;
              footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
               }
            //Compute and get the desired foot position and velocity
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingStates[foot], 0.25); //FIX: second argument not used in function
            
            // std::cout << "Foot " << foot << " swing state: " << swingStates[foot] << "\n";
            pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
            Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();

            pFoot_w_des[foot] = pDesFootWorld;

            double side = (foot == 0) ? 1.0 : -1.0; //Left foot (0) side = -1.0, Right foot (1) side = 1.0
            Eigen::Vector3d hipWidthOffSet = {-0.015, side*-0.055, 0.0}; // TODO: sync with Biped.h
            Vec3<double> hipOffset = {-0.025, side*0.05, 0};

            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position) - hipOffset ;
            vFoot_b[foot] = seResult.rBody * (vDesFootWorld - seResult.vWorld);   

        }
    }    
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeIK(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg){          

        Vec3<double> pFoot_des_b = bodyPositionDesired;
        double side = (leg == 0) ? -1.0 /*Left foot in swing*/ : 1.0 /*Right foot in swing*/;
        // hip_roll = {0.0465, 0.02*side, -0.197};

        Eigen::Vector3d hip_roll(L_hipRollLocation[0]-0.06, 0.0, L_hipYawLocation[2]+L_hipRollLocation[2]);
        // std::cout << "Hip Roll Location: " << hip_roll.transpose() << "\n";
        hip_roll = {0.0465, 0.02*side, -0.197};
        Eigen::Vector3d foot_des_to_hip_roll = pFoot_des_b - hip_roll; //in hip roll frame
        double distance_3D = foot_des_to_hip_roll.norm();
        double distance_2D_yOz = std::sqrt(std::pow(foot_des_to_hip_roll[1], 2) + std::pow(foot_des_to_hip_roll[2], 2));
        double distance_horizontal = 0.0205;
        double distance_vertical = std::sqrt(std::max(0.00001, std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2)));        // double distance_vertical = std::sqrt(std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2));
        double distance_2D_xOz = pow(( pow(distance_3D,2.0)-pow(distance_horizontal,2.0)), 0.5);
                       
        // Ensure arguments are within valid range for acos and asin
        double acosArg1 = clamp(distance_2D_xOz / (2.0 * 0.22), -1.0, 1.0);
        double acosArg2 = clamp(distance_vertical / distance_2D_xOz, -1.0, 1.0);
        double divisor = std::abs(foot_des_to_hip_roll[0]);
        divisor = (divisor == 0.0) ? 1e-6 : divisor; // Prevent division by zero

        // Joint angle calculations
        jointAngles[0] = 0.0; 
        jointAngles[1] = std::asin(clamp(foot_des_to_hip_roll[1] / distance_2D_yOz, -1.0, 1.0)) + std::asin(clamp(distance_horizontal * side / distance_2D_yOz, -1.0, 1.0));        
        jointAngles[2] = std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;
        jointAngles[3] = 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / 0.22, -1.0, 1.0)) - M_PI;
        jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2); // q3 - q2        

}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::setDesiredJointState(){
    for(int leg = 0; leg < nLegs; leg++){
        if(swingStates[leg] > 0){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, leg);
            data->_legController->commands[leg].qdDes = Eigen::Matrix<double, 5, 1>::Zero();
            Eigen::VectorXd kpgains(5);
            kpgains << 0, 0, 0, 0,  0;
            Eigen::VectorXd kdgains(5);
            kdgains << 1.5, 1.5, 1.5, 1.5, 1.5;
             data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
             data->_legController->commands[leg].pDes = pFoot_b[leg];
             data->_legController->commands[leg].vDes = vFoot_b[leg];
             data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
             data->_legController->commands[leg].kdJoint = kdgains.asDiagonal();             
             data->_legController->commands[leg].kptoe = 5; 
             data->_legController->commands[leg].kdtoe = 0.1;              
        }else{
            //Ensure no interference with stance leg controller
            Eigen::VectorXd kpgains(5);
            Eigen::VectorXd kdgains(5);
            kpgains.setZero();
            // kdgains.setZero();
            kdgains << 1.5, 1.5, 1.5, 1.5, 1.5;
            data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
            data->_legController->commands[leg].kdJoint = kdgains.asDiagonal(); 
            data->_legController->commands[leg].kpCartesian = Eigen::Matrix3d::Zero();
            data->_legController->commands[leg].kdCartesian = Eigen::Matrix3d::Zero();               
        }
    }
}