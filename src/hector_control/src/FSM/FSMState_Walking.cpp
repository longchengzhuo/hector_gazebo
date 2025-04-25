#include "../../include/FSM/FSMState_Walking.h"

FSMState_Walking::FSMState_Walking(ControlFSMData *data)
                 :FSMState(data, FSMStateName::WALKING, "walking"),
                  Cmpc(0.001, 40){}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_Walking::enter()
{
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
    _data->_armController->zeroCommand();
    Cmpc.firstRun = true;
    motiontime = 0;

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
    // start_time = 0;
}

void FSMState_Walking::run()
{                     
    _data->_legController->updateData(_data->_lowState);
    _data->_armController->updateData(_data->_lowState);
    _data->_stateEstimator->run(); 

    xAxis = _data->_joystickInterface->getAxisValue("LeftStickX");
    yAxis = _data->_joystickInterface->getAxisValue("LeftStickY");
    zAxis = _data->_joystickInterface->getAxisValue("RightStickY");

    // std::cout << "Left Y axis is " << _data->_joystickInterface->getAxisValue("LeftStickY") << std::endl;
    // std::cout << "Left X axis is " << _data->_joystickInterface->getAxisValue("LeftStickX") << std::endl;
    // std::cout << "Right Y axis is " << _data->_joystickInterface->getAxisValue("RightStickY") << std::endl;


    v_des_body[0] = xAxis * 0.5;  //x speed command max
    v_des_body[1] = -yAxis * 0.5; // y speed command max
    turn_rate = zAxis * 4;

    buttonA = _data->_joystickInterface->getButtonState("A");
    buttonB = _data->_joystickInterface->getButtonState("B");
    buttonX = _data->_joystickInterface->getButtonState("X");
    buttonY = _data->_joystickInterface->getButtonState("Y");
    buttonRB = _data->_joystickInterface->getButtonState("RightBumper");
    left_shoulder = _data->_joystickInterface->getButtonState("LeftTrigger");
    buttonback = _data->_joystickInterface->getButtonState("Back"); 

    // std::cout << "buttonA is " << buttonA << std::endl;
    // std::cout << "buttonB is " << buttonB << std::endl;
    // std::cout << "buttonX is " << buttonX << std::endl;
    // std::cout << "buttonY is " << buttonY << std::endl;

    _percent += 1.0 / _duration;
    _percent = _percent > 1 ? 1 : _percent;

    for (int arm = 0; arm < 2; arm++)
    {
        for (int j = 0; j < 4; j++)
         {
             _data->_armController->commands[arm].qDes(j) = (1 - _percent)*_armstartPos[arm*4+j]+_percent*_armtargetPos[arm*4+j];
         }
    }

    // if (buttonback) {
    //     std::cout << "keypad stop pressed" << std::endl;
    //     abort();
    // }

    int gaitTime = 200;

    if(buttonA){
        flagGaitTimer_Walk = 1;
    }
    if(flagGaitTimer_Walk == 1 && motiontime%gaitTime == gaitTime/2){
        flagWalk = 1;
        flagGaitTimer_Walk = 0;
    }
    if(buttonB){
        flagGaitTimer_Stand = 1;
    }
    if(flagGaitTimer_Stand == 1 && motiontime%gaitTime == 0){
        flagWalk = 0;
        flagGaitTimer_Stand = 0;
    }


    int gaitNum;
    if(flagWalk == 0){
        gaitNum = 7;
    }

    if(flagWalk == 1){
        gaitNum = 3;
    }

    double percent;

    Cmpc.setGaitNum(gaitNum); // 2 for walking
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    Cmpc.run(*_data);

    //manual arm motions:
    // for(int i = 0; i<8; i++){
    //     qdes_arm[i] = qdes_nominal[i];
    // }
    // for(int i = 0; i<2; i++){ //heuristic swing
    //     qdes_arm[i*4+1] += _data->_legController->data[i].q(2)-0.78;
    //     qdes_arm[i*4+2] += _data->_legController->data[i].q(1);
    // }

    // if(buttonY){ //do waving for a while
    //     waveTimer = motionTime;
    //     std::cout << "Waving" << std::endl;
    // }
    // if(waveTimer +500 > motionTime && waveTimer > 0){ // bring up hand
    //     percent = (motionTime - waveTimer)/500.0;
    //     std::cout << "percent is " << percent << std::endl;
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_nominal[i] + (qdes_up[i]-qdes_nominal[i])*percent;
    //     }
    // }
    // double waveTime = 3000;
    // if(waveTimer + waveTime+500 > motionTime && waveTimer + 500 < motionTime && waveTimer > 0){ // wave hand
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_up[i];
    //     }
    //     qdes_arm[7] += 0.3*sin(motionTime/100.0);
    // }
    // if(waveTimer + waveTime+1000 > motionTime && waveTimer + waveTime+500 < motionTime && waveTimer > 0){ // bring down hand
    //     percent = (motionTime - int(waveTimer+waveTime+500))/500.0;
    //     // std::cout << "motionTime: "<< motionTime << " waveTimer: " << waveTimer << "percent: " << percent<<std::endl;
    //     // percent = 1;
    //     // std::cout<<"percent: "<<percent<<std::endl;
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_up[i] + (qdes_nominal[i]-qdes_up[i])*percent;
    //     }
    //     if(waveTimer+1999 == motionTime){
    //         waveTimer = 0;
    //     }
    // }
    // //hiFive
    // if(buttonRB && hiFiveSeq < 3 && motionTime - hiFiveTimer > 200 ){
    //     hiFiveTimer = motionTime;
    //     hiFiveSeq ++; 
    // }
    // if(hiFiveSeq == 1){
    //     percent = 1.0;
    //     if(motionTime - hiFiveTimer < 500){
    //         percent = (motionTime - hiFiveTimer)/500.0;
    //     }
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_nominal[i] + (qdes_hiFiveUp[i]-qdes_nominal[i])*percent;
    //     }
    // }
    // if(hiFiveSeq == 2){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_hiFive[i];
    //     }
    // }
    // if(hiFiveSeq == 3){
    //     percent = 1.0;
    //     if(motionTime - hiFiveTimer < 500){
    //         percent = (motionTime - hiFiveTimer)/500.0;
    //     }
    //     if(motionTime - hiFiveTimer > 500){
    //         hiFiveSeq = 0;
    //     }
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_hiFive[i] + (qdes_nominal[i] - qdes_hiFive[i])*percent;
    //     }
    // }

    // // move boxes
    // if(buttonX && pickupSeq < 6 && motionTime - pickupTimer > 200 ){
    //     pickupTimer = motionTime;
    //     pickupSeq ++; 
    // }
    // if(pickupSeq == 1){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_prepare[i];
    //     }
    // } 
    // if(pickupSeq == 2){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_clamp[i];
    //     }
    // } 
    // if(pickupSeq == 3){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_hold[i];
    //     }
    // } 
    // if(pickupSeq == 4){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_clamp[i];
    //     }
    // } 
    // if(pickupSeq == 5){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_prepare[i];
    //     }
    // } 
    // if(pickupSeq == 6){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] = qdes_nominal[i];
    //     }
    //     pickupSeq = 0;
    // } 

    // //dab
    // if (yAxis > 0 && gaitNum == 7){
    //     for(int i = 0; i<8; i++){
    //         qdes_arm[i] =  qdes_nominal[i] + yAxis*(qdes_dab_l[i] - qdes_nominal[i]);
    //     }
    // }
    
    // //send commands to arms:
    // double Kp_arm[8] = {5,5,5,5, 5,5,5,5};
    // double Kd_arm[8] = {0.1,0.1,0.1,0.1, 0.1,0.1,0.1,0.1};
    // if(pickupSeq > 0){
    //     for(int i = 0; i<8; i++){
    //         Kp_arm[i] *= 2.0;
    //         Kd_arm[i] *= 3.0;
    //         // std::cout << "Kp " << Kp_arm[i] <<" Kd " << Kd_arm[i] <<std::endl; 
    //     }
    // }
    // for(int i = 0; i<4; i++){
    //     _data->_armController->commands[0].qDes(i) = qdes_arm[i];
    //     _data->_armController->commands[1].qDes(i) = qdes_arm[i+4];
    //     _data->_armController->commands[0].kpJoint(i,i) = Kp_arm[i];
    //     _data->_armController->commands[1].kpJoint(i,i) = Kp_arm[i+4];
    //     _data->_armController->commands[0].kdJoint(i,i) = Kd_arm[i];
    //     _data->_armController->commands[1].kdJoint(i,i) = Kd_arm[i+4];
    // }


    _data->_legController->updateCommand(_data->_lowCmd, motiontime);
    _data->_armController->updateCommand(_data->_lowCmd);
    motiontime++;
}

void FSMState_Walking::exit()
{      
}

FSMStateName FSMState_Walking::checkTransition()
{       //TODO: Implement transition logic
    // int gaitTime = 10 * mpcdt / 2;
    // buttonA = _data->_joystickInterface->getButtonState("A");
    // buttonB = _data->_joystickInterface->getButtonState("B");

    //     if(buttonA){
    //         flagGaitTimer_Walk = 1;
    //     }
    //     if(flagGaitTimer_Walk == 1 && counter%gaitTime == gaitTime/2){
    //         flagWalk = 1;
    //         flagGaitTimer_Walk = 0;
    //     }
    //     if(buttonB){
    //         flagGaitTimer_Stand = 1;
    //     }
    //     if(flagGaitTimer_Stand == 1 && counter%gaitTime == 0){
    //         flagWalk = 0;
    //         flagGaitTimer_Stand = 0;
    //     }

    //     if(_data->_joystickInterface->getButtonState("LeftBumper")){
    //         abort();
    //         return FSMStateName::PASSIVE;
    //     }else if(flagWalk == 0){
    //         gait_num = 1;
    //         Cmpc.firstRun = true;
    //         return FSMStateName::WALKING;
    //     }else if(flagWalk){
    //         gait_num = 2;
    //         Cmpc.firstRun = true;            
    //         return FSMStateName::WALKING;
    //         }
    //     return FSMStateName::WALKING;

        if(_data->_joystickInterface->getButtonState("LeftBumper")){
            abort();
            return FSMStateName::PASSIVE;
        }else if(_data->_joystickInterface->getButtonState("B")){
            gait_num = 1;
            Cmpc.firstRun = true;
            return FSMStateName::WALKING;
        }else if(_data->_joystickInterface->getButtonState("A")){
            gait_num = 2;
            Cmpc.firstRun = true;            
            return FSMStateName::WALKING;
            }
        return FSMStateName::WALKING;
}

// Eigen::MatrixXd FSMState_Walking::readDataFromFile(const std::string& filePath) {
//     std::vector<std::vector<double>> data;
//     std::ifstream myFile(filePath);

//     // If cannot open the file, report an error
//     if (!myFile.is_open()) {
//         throw std::runtime_error("\nERROR: Could not open file: " + filePath);
//     }

//     std::cout << "Reading Optimization Data for Position from: " << filePath << std::endl;
//     std::string line;

//     // Read data line by line
//     while (getline(myFile, line)) {
//         std::stringstream ss(line);
//         double val;
//         std::vector<double> row;
//         while (ss >> val) {
//             row.push_back(val);
//             if (ss.peek() == ',') ss.ignore();
//         }
//         data.push_back(row);
//     }

//     myFile.close();

//     // Convert to Eigen type
//     if (data.size() == 0) return Eigen::MatrixXd(); // Check for empty data
//     size_t numRows = data.size();
//     size_t numCols = data[0].size();
//     Eigen::MatrixXd mat(numRows, numCols);
//     for (size_t i = 0; i < numRows; ++i) {
//         for (size_t j = 0; j < numCols; ++j) {
//             mat(i, j) = data[i][j];
//         }
//     }

//     return mat.transpose(); 
// }

// void FSMState_Walking::extractTrajectory(const Eigen::MatrixXd& trajectory, double time, Eigen::VectorXd& track, double dataFrequency) {

//     int index = static_cast<int>(floor(time * dataFrequency/1000.0));
//     // std::cout << "index: " << index << std::endl;
    
//     // Check if the index is within the bounds of the trajectory matrix
//     if (index < 0 || index >= trajectory.rows()) {
//         // std::cerr << "Error: Time index is out of bounds." << std::endl;
//         track = trajectory.row(trajectory.rows()-1).transpose(); //set track to last row of trajectory
//         return;
//     }
//     track = trajectory.row(index).transpose();
//     // std::cout << "ref: \n" <<  track << std::endl;
// }