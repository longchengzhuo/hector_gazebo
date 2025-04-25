#include <iostream>
#include "../include/common/Utilities/Timer.h"
// #include "../include/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
// #include "../include/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
// #include "../include/body.h"

using namespace ori;
// using namespace laikago_model;


/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),
                                                                                    standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  // gaitNumber = 7;
  dtMPC = dt * iterationsBetweenMPC;
  // std::cout << "dtMPC: " << dtMPC << std::endl;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
    // std::cout <<"Constructor"<<std::endl;
  // foot_position.open("foot_pos.txt");

}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;

  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  Gait *gait = &standing;
  if (gaitNumber == 7)
    gait = &standing;
  else if (gaitNumber == 3)
    gait = &walking;

  current_gait = gaitNumber;

  Vec3<double> v_des_robot;
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.55; //.5;;;

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
    * (data._biped->getHipYawLocation(i) + data._legController->data[i].p);
  }
    // std::cout << "pFoot 0: \n" << pFoot[0] << std::endl;
    // std::cout << "pFoot 1: \n" << pFoot[1] << std::endl;
      // swing.initSwingLegController(&data, gait, dtMPC);

  double swingHeight = 0.08;
  // some first time initialization
  if (firstRun)
  {
    swing.initSwingLegController(&data, gait, dtMPC);
    // std::cout << "***********First Run***********\n\n\n\n" << std::endl;

    // std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    //
    if (gaitNumber == 7)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = 0.55;

      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(swingHeight);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }


  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;

  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 2; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }


      footSwingTrajectories[i].setHeight(swingHeight);
      Vec3<double> offset(0.0, side_sign[i] * 0.0, 0.0);
      // simple heuristic function


      Vec3<double> pRobotFrame = (data._biped->getHipYawLocation(i) + offset);
      Vec3<double> des_vel;

      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);

      Vec3<double> Pf = seResult.position +
                        seResult.rBody.transpose() * pRobotFrame 
                        + seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.3;
      double pfx_rel = 1.0*seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                       0.01 * (seResult.vWorld[0] - v_des_world[0]);

      double pfy_rel = 1.0 * seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                       0.01 * (seResult.vWorld[1] - v_des_world[1]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      // std:: cout << "pfy_rel =" << pfy_rel << "\n";
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = 0.0;

      footSwingTrajectories[i].setFinalPosition(Pf);
    // }
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter); 
  // load LCM leg swing gains
  Kp << 250, 0, 0,
      0, 250, 0,
      0, 0, 200;
  Kp_stance = 0* Kp;

  Kd << 5, 0, 0,
      0, 5, 0,
      0, 0, 5;
  Kd_stance = 0*Kd;
  // gait
  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();

  if (gaitNumber == 7)
  {
    data._lowCmd->MPC_phase[0] =1.0;
    data._lowCmd->MPC_phase[1] =1.0;
  } else {
    data._lowCmd->MPC_phase[0] = swingStates[0];
    data._lowCmd->MPC_phase[1] = swingStates[1];
  }
  // std::cout << "MPC phase is " << data._lowState->MPC_phase[0] << " " << data._lowState->MPC_phase[1] << std::endl;

  swing_state__ = swingStates;
  contact_states__ = contactStates; 
  // std::cout << "********MPC swing state: " << swingStates << std::endl; 

  int *mpcTable = gait->mpc_gait();
  
  updateMPCIfNeeded(mpcTable, data, omniMode);

  iterationCounter++;

  Vec2<double> se_contactState(0, 0);

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot); 
    // std::cout << "swing" << foot << ": " << swingState << std::endl;
    // std::cout << "Contact" << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        // std::cout << "check 1" << std::endl;
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        pFootWorld = pFoot[foot];
        // footSwingTrajectories[foot].setHeight(0.1);
      }

      // std::cout << "foot" << foot << ": " << foot << std::endl;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      double side = -1.0 ;
      if (foot == 1){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = -1.0;}
    else if (foot == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
      Vec3<double> dummyPos = {seResult.position[0],seResult.position[1],seResult.position[2]};
      Vec3<double> dummyVel = {seResult.vWorld[0],seResult.vWorld[1],seResult.vWorld[2]};
      Vec3<double> hipOffset = {0.025, side*0.045, -0.136*0};
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - dummyPos) - hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - dummyVel);
      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }


    //   // new trajectory:
    //    Vec3<double> pDesFootWorld = {0, 0, 0};
    //   Vec3<double> vDesFootWorld = {0, 0, 0};
    //   double side; // 1 for Left legs; -1 for right legs
    // if (foot == 1){
    //     // std::cout<< "Leg Sign checked" << std::endl;
    //     side = -1.0;}
    // else if (foot == 0){
    //     // std::cout<< "Leg Sign checked" << std::endl;
    //     side = 1.0; //0.5
    // }
    //   double footHeight = 0.09;
    //   pDesFootWorld[0] =  seResult.position[0] +
    //                   (1.0*seResult.vWorld[0] * 0.50 * gait->_stance * dtMPC +
    //                    0.05 * (seResult.vWorld[0] - v_des_world[0]) )*swingState;
    //   pDesFootWorld[1] = seResult.position[1] + 
    //                   (1.00*seResult.vWorld[1] * 0.50 * gait->_stance * dtMPC +
    //                    0.05 * (seResult.vWorld[1] - v_des_world[1])) * swingState;
    //   // pDesFootWorld[0] = -0.05 + seResult.position[0] +
    //   //                 (1.0*seResult.vWorld[0] * 0.50 * gait->_stance * dtMPC +
    //   //                  0.05 * (seResult.vWorld[0] - v_des_world[0]) );
    //   // pDesFootWorld[1] = seResult.position[1] + 
    //   //                 (1.00*seResult.vWorld[1] * 0.50 * gait->_stance * dtMPC +
    //   //                  0.05 * (seResult.vWorld[1] - v_des_world[1]));
    //   // pDesFootWorld[0] = swingState * (seResult.position[0] +
    //   //                 1.0*seResult.vWorld[0] * 0.50 * gait->_stance * dtMPC +
    //   //                 0.02 * (seResult.vWorld[0] - v_des_world[0])) +   
    //   //                 (1.0 - swingState) * pFootWorld[0];
    //   // pDesFootWorld[1] = swingState * (seResult.position[1] + 
    //   //                 1.0 * seResult.vWorld[1] * 0.50 * gait->_stance * dtMPC +
    //   //                 0.02 * (seResult.vWorld[1] - v_des_world[1])) +  
    //   //                 (1.0 - swingState) *pFootWorld[1];
    //   pDesFootWorld[2] = -footHeight/2.0 * std::cos(2.0*3.1415*swingState)+footHeight/2 - 0.0;
    //   vDesFootWorld[2] = footHeight * 3.1415 * std::sin(2.0*3.1415*swingState);

    //   Vec3<double> hipHeightOffSet = {0,0,-0.136*0.0};
    //   Vec3<double> hipWidthOffSet = {-0.04,side*-0.0,0};
    //   // Vec3<double> dummyPos = {seResult.position[0],seResult.position[1], 0.55};
    //   // Vec3<double> dummyVel = {seResult.vWorld[0],seResult.vWorld[1],0.0};
    //   Vec3<double> dummyPos = {seResult.position[0],seResult.position[1], seResult.position[2]};
    //   Vec3<double> dummyVel = {seResult.vWorld[0],seResult.vWorld[1], seResult.vWorld[2]};
      
    //   Vec3<double> pDesLeg =  seResult.rBody * (pDesFootWorld - dummyPos) - hipHeightOffSet + hipWidthOffSet;
    //   Vec3<double> vDesLeg =  seResult.rBody * (vDesFootWorld - dummyVel);

    //   if (vDesLeg.hasNaN())
    //   {
    //     vDesLeg << 0, 0, 0;
    //   }
      if (pDesLeg.hasNaN())
      {
        pDesLeg << 0, 0, -0.4;
      }
      // std::cout << "pDesKeg: " << pDesLeg[0] << "," << pDesLeg[1] << "," << pDesLeg[2] << std::endl;
      data._legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      // std::cout << "check 3" << std::endl;
      data._legController->commands[foot].kptoe = 10; // 0
      data._legController->commands[foot].kdtoe = 0.2;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(0) << std::endl;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(1) << std::endl;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(2) << std::endl;
      // std::cout << "foot Des " << foot << ": \n " << pDesLeg(2) << std::endl;
      // singularity barrier
      // data._legController->commands[foot].tau[2] =
      //  50*(data._legController->data[foot].q(2)<.1)*data._legController->data[foot].q(2);
      // std::cout << "contat " << foot << ": " << contactState << std::endl;
      // if (foot == 0)
      // {
      //   foot_position << pDesFootWorld[0] << " " << pDesFootWorld[1] << " " << pDesFootWorld[2] << "\n";
      // }
      // if(climb){
      //   if(lowState.footForce[foot] > 10 && swingState>0.5){
      //     //std::cout << "force changed for leg " << foot << std::endl;
      //     data._legController->commands[foot].kpCartesian = Kp_stance;
      //     data._legController->commands[foot].kdCartesian = 0 * Kd;
      //     data._legController->commands[foot].feedforwardForce << 0, 0, -10;
      //     contactState = 0;
      //     firstSwing[foot] = true;
      //   }
      // }
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    { 
      firstSwing[foot] = true;
      // footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      // Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      // Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHip2Location(foot);
      // Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // if (vDesLeg.hasNaN())
      //   {
      //    vDesLeg << 0, 0, 0;
      //    }

      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
      // cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      // std::cout << "robot pos: \n" << seResult.position << std::endl;
      // foot_swing << pDesFootWorld[2] << " ";
      Vec3<double> pDesLeg = {0, 0, -0.4};
      Vec3<double> vDesLeg = {0, 0, 0};
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];
      // std::cout << "y: " << pDesLeg[1] << std::endl;
      // std::cout << "contact " << foot << " : \n" << contactState << std::endl;
      // std::cout << "foot force " << foot << " : \n " << data._legController->commands[foot].feedforwardForce(2) << std::endl;
      // data._legController->commands[foot].kdJoint = Mat3<double>::odyIdentity() * 0.2

      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      // Fr_des[foot] = -f_ff[foot];

      // foot_swing << "\n";
      // std::cout << "foot Des" << foot << " \n " << data._legController->commands[foot].pDes << std::endl;
    }

    // se->set_contact_state(se_contactState); todo removed
    // data._stateEstimator->setContactPhase(se_contactState);
    // data._legController->updateCommand();
  }

  // TODO fix IK
  // for (int foot = 0; foot < 2; foot++)
  // {

  //   double contactState = contactStates(foot);
  //   double swingState = swingStates(foot); 

  //   Vec3<double> pFootWorld;
  //   swing.updateSwingLeg(); // update swing leg controller

  //   swingTraj << swing.pDesFootWorld[0] << " " << swing.pDesFootWorld[1] << " " << swing.pDesFootWorld[2] << " " << swing.pFoot_b[0][0] << " " << swing.pFoot_b[0][1] << " " << swing.pFoot_b[0][2] << "\n";
  //   // log swingState, swingTimes[foot]
  //   std::cout <<"Foot " << foot << " CMPC swingState: " << swingState << std::endl;
  //   std::cout << "CMPC swingTime: " << swingTimes[foot] << std::endl;

  //   if (swingState > 0) // foot is in swing
  //   {
  //     se_contactState[foot] = contactState;
  //   }

  //   else if (contactState > 0) // foot is in stance
  //   { 
  //     firstSwing[foot] = true;
  //     Vec3<double> pDesLeg = {0, 0, 0};
  //     Vec3<double> vDesLeg = {0, 0, 0};
  //     data._legController->commands[foot].pDes = pDesLeg;
  //     data._legController->commands[foot].vDes = vDesLeg;
  //     data._legController->commands[foot].kpCartesian = Kp_stance; // 0
  //     data._legController->commands[foot].kdCartesian = Kd_stance;
  //     data._legController->commands[foot].kptoe = 0; // 0
  //     data._legController->commands[foot].kdtoe = 0;
  //     data._legController->commands[foot].feedforwardForce = f_ff[foot];
  //     se_contactState[foot] = contactState;

  //     // Update for WBC
  //     // Fr_des[foot] = -f_ff[foot];
  //   }

  //   data._stateEstimator->setContactPhase(se_contactState);
  // }
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{

  if ((iterationCounter % 5) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *q = seResult.orientation.data();

    double r[6];
    // std::cout << "r_foot****** : \n";
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
      // std::cout << r[i] << std::endl; 

    }
    
      double Q[12] = {250, 500, 100,   600, 600, 800,   .1, .1, 1,   .1, .1, .1}; //original icra gains
      // double Q[12] = {200, 200, 200,   200, 200, 200,   1, 1, 1,   1, 1, 1}; // nominal initial
      // // traj 1:
      // double Q[12] = {203.1617,  255.1576 , 216.1241,  310.2656,  284.3867,  243.8494,    0.6302 ,   0.6302  ,  0.6302  ,  1.5513  ,  1.5513 ,   1.5513};
      // // traj 2:
      // double Q[12] = {197.1058,  203.1815,  165.8102,  200.8021,  140.4742,  204.1452 ,   1.0395 ,   0.6302  ,  1.5513  ,  1.5513  ,  0.6302  ,  1.5513 };
      // // traj 3:
      // double Q[12] = {151.4287 , 172.3033 , 138.9805 , 204.1441 , 279.7455 , 127.3322  ,  0.8510  ,  1.5513  ,  1.5513  ,  0.6302  ,  1.4036  ,  0.6302 };

      // double Q[12] = {287.6, 95.65, 380.07, 116.91, 355.17, 167.4, 1.59, 1.94, 0.47, 0.87, 0.87, 1.06};
      // double Q[12] = {191.75, 194.81, 100.47,  186.29, 134.94, 669.01,  0.1216, 0.1216, 0.1518,  0.6727, 0.10, 0.6727};

    // std::cout << "gait " << gaitNumber <<std::endl;


    // double Alpha[12] = {1e-4, 1e-4, 1e-3, 1e-4, 1e-4, 1e-3,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2};
    // double Alpha[12] = {0.166e-3, 0.0673e-3, 0.0673e-3, 0.1428e-3, 0.0673e-3, 0.1057e-3, 0.1659e-3, 0.1428e-3, 0.1428e-3, 0.0673e-3, 0.0783e-3, 0.0673e-3};;
    //  double Alpha[12] = {0.1651e-3, 0.2018e-3, 0.6727e-3, 0.6727e-3, 0.3685e-3, 0.6727e-3, 0.6727e-3, 0.3015e-3, 0.0122e-3, 0.1351e-3, 0.6727e-3, 0.1351e-3};
    // //handtune
    double Alpha[12] = {1e-4, 1e-4, 1e-3, 1e-4, 1e-4, 1e-3,   2e-2, 2e-2, 2e-2, 2e-2, 2e-2, 2e-2};
    // //nominal 
    // double Alpha[12] = {1e-5, 1e-5, 1e-5,  1e-5, 1e-5, 1e-5,   1e-5, 1e-5, 1e-5,  1e-5, 1e-5, 1e-5};
    // //traj 1: 
    // double Alpha[12] = {0.1053e-3 ,   0.1999e-3  ,  0.1999e-3  ,  0.0472e-3  ,  0.1999e-3 ,   0.1999e-3  ,  0.1000e-3  ,  0.1999e-3  ,  0.0472e-3  ,  0.1000e-3  ,  0.1999e-3  ,  0.1999e-3};
    // //traj 2: 
    // double Alpha[12] = {0.1053e-3 ,   0.1053e-3 ,   0.0472e-3 ,   0.1999e-3 ,   0.0472e-3 ,   0.1999e-3 ,   0.0472e-3 ,   0.0472e-3 ,   0.0472e-3 ,   0.0651e-3  ,  0.0472e-3 ,   0.0897e-3};
    // //traj 3: 
    // double Alpha[12] = {0.1053e-3 ,   0.1703e-3  ,  0.0472e-3,   0.1999e-3 ,   0.0554e-3  ,  0.0897e-3  ,  0.1451e-3  ,  0.0651e-3  ,  0.0554e-3  ,  0.0764e-3  ,  0.0897e-3  ,  0.0651e-3};
    

    //// baseline test (no actnet)
    // traj 1:
    // double Q[12] = {200.1754,  200.9652,  200.0122,  310.2656,  269.7654 , 232.2682,    1.5513 ,   0.6302 ,   0.6772,    1.5513 ,   1.5513 ,   1.5513};
    // double Alpha[12] = {0.1999e-3,  0.1999e-3,    0.1999e-3,    0.0472e-3,    0.1236e-3,    0.1999e-3,    0.1000e-3,    0.1999e-3,    0.0472e-3,    0.1000e-3,    0.1999e-3,    0.1999e-3};
    // traj 2:
    // double Q[12] = {209.0617,  200.8865,  183.9700 , 185.5144,  160.0986,  200.8309,    0.6302,    0.6302,    1.5513,    1.5513,    0.6966,    1.0395};
    // double Alpha[12] = {0.1451e-3,    0.0472e-3,    0.0472e-3,    0.1999e-3,    0.0472e-3,    0.1999e-3,    0.0472e-3,    0.0472e-3,    0.0472e-3,    0.0472e-3,    0.0472e-3,    0.0472e-3};
    // traj 3:
    // double Q[12] = {205.8046,  188.5926,  274.6056,  209.6342,  221.5708 , 160.0366,    1.4036 ,   1.1490,    0.6302,    0.6302,    1.5513 ,   0.6966};
    // double Alpha[12] = {0.0472e-3,    0.0764e-3,    0.0472e-3,    0.1703e-3,    0.0472e-3,    0.1999e-3,    0.0764e-3,    0.0651e-3,    0.1703e-3,    0.1999e-3,    0.1451e-3,    0.0472e-3};

    double *weights = Q;
    double *Alpha_K = Alpha;

 
    double yaw = seResult.rpy[2];

    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0);

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;

    const double max_pos_error = 0.05;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    double max_yaw_error = 0.1;
    if (yaw - yaw_desired > max_yaw_error) yaw_desired = yaw;
    if (yaw_desired - yaw > max_yaw_error) yaw_desired = yaw;


    double yaw_des = v_des_robot[1] * 6;
    double height_add_des = v_des_robot[0] * 0.5;

    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    double yaw_rate_des = 0.0;
    double roll_comp = 0.0;
    double pitch_comp = 0.0;
    Vec3<double> foot_center = (pFoot[0]+pFoot[1])/2.0;
    // std::cout << "pFoot 0: " << pFoot[0] << std::endl;
    // std::cout << "pFoot 1: " << pFoot[1] << std::endl;
    
    if (gaitNumber == 7) { //standing
      v_des_world[0] = 0;
      v_des_world[1] = 0;
      roll_comp = v_des_robot[0]*0.25;
      pitch_comp = v_des_robot[1]*0.5;
      // xStart = foot_center[0] - 0.0;
      // yStart = foot_center[1];
    }
    else { // with gait
      height_add_des = 0;
      yaw_rate_des = v_des_world[1] * 5.0;
      roll_comp = -stateCommand->data.stateDes[11]/40.0;
      pitch_comp = v_des_robot[0]*0.15*0;
      // xStart = foot_center[0] + 0.015;
      // yStart = foot_center[1];
    }
    // std::cout << "xStart: " << xStart << std::endl;
    // std::cout << "stateDes xy: " << stateCommand->data.stateDes[0] << " " << stateCommand->data.stateDes[1] << std::endl;
    double trajInitial[12] = {roll_comp + stateCommand->data.stateDes[3]*0,  // 0
                              0.0,    // 1
                              yaw_desired,    // 2
                              xStart*1.0,                                   // 3
                              yStart*1.0,                                   // 4
                              0.55 + height_add_des*0,   // 5
                              0,                                        // 6
                              0,                                        // 7
                              stateCommand->data.stateDes[11],  // 8
                              v_des_world[0],                           // 9
                              v_des_world[1],                           // 10
                              0};   // 11
    // double trajInitial[12] = {roll_comp + stateCommand->data.stateDes[3]*0,  // 0
    //                           pitch_comp*0-0.0,    // 1
    //                           seResult.rpy[2]*0.0,    // 2
    //                           stateCommand->data.stateDes[0],                                   // 3
    //                           stateCommand->data.stateDes[1],                                   // 4
    //                           0.57 + height_add_des*0,   // 5
    //                           0,                                        // 6
    //                           0,                                        // 7
    //                           stateCommand->data.stateDes[11],  // 8
    //                           0,                           // 9
    //                           v_des_world[1],                           // 10
    //                           0};   // 11
    

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if (v_des_world[0] < 0.01 && v_des_world[0] > -0.01) {
        trajAll[12*i + 3] = trajInitial[3];
        }
        else{
         trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0]; 
        }
        if (v_des_world[1] < 0.01 && v_des_world[1] > -0.01) {
        trajAll[12*i + 4] = trajInitial[4];
        }
        else{
         trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1]; 
        }
        if (stateCommand->data.stateDes[11] < 0.02 && stateCommand->data.stateDes[11] > -0.02){
        trajAll[12*i + 2] = trajInitial[2];
         }
        else{
        trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      // }
      // std::cout << "traj " << i << std::endl;
      // for (int j = 0; j < 12; j++) {
      //   std::cout << trajAll[12 * i + j] << "  ";
      // }
      //     std::cout<< " " <<std::endl;
    }
    // }

    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 0.25, 500);
    Timer t2;
    t2.start();
    // cout << "dtMPC: " << dtMPC << "\n";
    //  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
    update_problem_data(p, v, q, w, r, yaw, weights, trajAll, Alpha_K, mpcTable, data);

    // for (int i = 0; i < 3; i++){
    //   mpc_input << p[i] << "  ";
    // }
    // for (int i = 0; i < 3; i++){
    //   mpc_input << v[i] << "  ";
    // }
    // for (int i = 0; i < 4; i++){
    //   mpc_input << q[i] << "  ";
    // }
    // for (int i = 0; i < 3; i++){
    //   mpc_input << w[i] << "  ";
    // }
    // for (int i = 0; i < 6; i++){
    //   mpc_input << r[i] << "  ";
    // }
    // mpc_input << std::endl;
    // std::cout << "p is " << std::endl;
    // for (int i = 0; i < 3; i ++)
    // {
    //   std::cout << p[i] << "  ";
    // }
    // std::cout << std::endl;
    // std::cout << "v is " << std::endl;
    // for (int i = 0; i < 3; i ++)
    // {
    //   std::cout << v[i] << "  ";
    // }
    // std::cout << std::endl;
    // std::cout << "q is " << std::endl;
    // for (int i = 0; i < 4; i ++)
    // {
    //   std::cout << q[i] << "  ";
    // }
    // std::cout << std::endl;
    // std::cout << "w is " << std::endl;
    // for (int i = 0; i < 3; i ++)
    // {
    //   std::cout << w[i] << "  ";
    // }
    // std::cout << std::endl;
    // std::cout << "r is " << std::endl;
    // for (int i = 0; i < 6; i ++)
    // {
    //   std::cout << r[i] << "  ";
    // }
    // std::cout << std::endl;
    // t2.stopPrint("Run MPC");
    printf("MPC Solve time %f ms\n", t2.getMs());
    // std::cout << t2.getSeconds() << std::endl;
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {

        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      GRF_R = -seResult.rBody * GRF;
    
      GRM_R = -seResult.rBody * GRM;
      // std::cout << "RBody: " << seResult.rBody << std::endl;
      // std::cout << "\n **RPY convex= \n"<<  ori::rotationMatrixToRPY(seResult.rBody) << std::endl;
      for (int i = 0; i < 3; i++){
        f(i) = GRF_R(i);
        f(i+3) = GRM_R(i);
      }
      f_ff[leg] = f;
      // f_ff[leg].setZero();

      // std::cout << f_ff[leg] << std::endl;
      // std::cout << "mpc solution" << leg << "\n" << f << std::endl;
      //  Update for WBC
      //  Fr_des[leg] = f;
    }
    contact_state(0) = mpcTable[0];
    contact_state(1) = mpcTable[1];
    // printf("update time: %.3f\n", t1.getMs());
  }
}