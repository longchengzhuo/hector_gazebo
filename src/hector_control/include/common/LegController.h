/*!
 * @file LegController.h
 * @brief Comman Leg Control Interface

 */ 

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
// #include "../../include/common/SwingLegController.h"
#include "Biped.h"
#include <fstream>
#include <chrono>


/*!
 * Data sent from control algorithm to legs
 */ 
    struct LegControllerCommand{
        LegControllerCommand() {zero();}

        void zero();

        Vec5<double> qDes, qdDes, tau;
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;
    };

/*!
 * Data returned from legs to control code
 */ 
    struct LegControllerData{
        LegControllerData() {zero();}
        void setBiped(Biped& biped) { hector = &biped; }

        void zero();
        Vec5<double> q, qd;
        Vec3<double> p, v;
        Mat65<double> J;
        Mat35<double> J2;
        Vec5<double> tau;
        Biped* hector;
    };

/*!
 * Controller for 2 legs of hector
 */ 
    class LegController {
      public:
        LegController(Biped& biped) : _biped(biped) {
            for (auto& dat : data) dat.setBiped(_biped);
            for(int i = 0; i<2; i++){
                commands[i].zero();
                data[i].zero();
            }
        };
        
        void zeroCommand();
        void edampCommand(double gain);
        void updateData(const LowlevelState* state);
        void updateCommand(LowlevelCmd* cmd, int motionTime);
        void setEnabled(bool enabled) {_legsEnabled = enabled;};  


        std::ofstream tau_leg_controller;
        std::ofstream feedforward_force;
        std::ofstream feedback_torque;
        std::ofstream kp_leg;
        std::ofstream desired_position;

        Vec5<double> Kp_joint = {0, 0, 0, 0, 0};
        Vec5<double> Kd_joint = {0, 0, 0, 0, 0};

        LegControllerCommand commands[2];
        LegControllerData data[2];
        bool __simulated_robot;
        bool _legsEnabled = false;
        std::string limbName[5] = {"Hip 1", "Hip 2", "Thigh", "Knee", "Toe"};
        std::string side[2] = {"Left", "Right"};
        Biped& _biped;
        //CurrentState& curr;
        //ros::NodeHandle n;
        int counter = 0;

        // double gear_ratio = 1.545;
        double gear_ratio = 2.0;
        static std::chrono::high_resolution_clock::time_point lastUpdateTime;
    };

    void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2,
                                       Vec3<double>* p, int leg, bool simulated_robot);

    // void computeInverseKinematics(Quadruped& _quad, Vec3<double>& pDes, int leg, Vec3<double>* qDes);

#endif