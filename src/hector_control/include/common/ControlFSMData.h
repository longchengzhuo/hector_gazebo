#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "LegController.h"
#include "ArmController.h"
#include "Biped.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "../interface/Interface.h"
#include "StateEstimatorContainer.h"
// #include "ArmLowLevel.h"
#include "../interface/Joystick_Interface.h"


struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Biped *_biped;
  StateEstimatorContainer *_stateEstimator;
  LegController *_legController;
  ArmController *_armController;
  DesiredStateCommand *_desiredStateCommand;
  Interface *_interface;
  // CheatIO *_interface;
  // IOSDK *_interface;
  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;
  JoystickInterface *_joystickInterface;
  // void sendRecv(){
  //   _interface->sendRecv(_lowCmd, _lowState);
  // }
};


#endif  // CONTROLFSM_H