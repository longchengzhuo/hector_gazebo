#include "hector_gazebo_fortress_plugin/hector_gazebo_fortress_plugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

// *** NECESSARY FOR LOGGING ***
#include <ignition/common/Console.hh> // <-- Ensure header is included and found

// ROS 2 Includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

// Standard Library Includes
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

namespace hector_gazebo_plugins {
    HectorGazeboFortressPlugin::HectorGazeboFortressPlugin()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    HectorGazeboFortressPlugin::~HectorGazeboFortressPlugin()
    {
        rosNode_.reset();
    }

    void HectorGazeboFortressPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                               const std::shared_ptr<const sdf::Element> &_sdf,
                                               ignition::gazebo::EntityComponentManager &_ecm,
                                               ignition::gazebo::EventManager &/*_eventMgr*/)
    {
        this->modelEntity_ = _entity;
        ignition::gazebo::Model model(this->modelEntity_);

        // --- 获取模型名称 ---
        // model.Name() 返回 std::optional<std::string>
        std::optional<std::string> modelNameOpt = model.Name(_ecm);

        // --- 从 std::optional<std::string> 安全地获取 std::string ---
        // 如果 modelNameOpt 包含值，则使用该值；否则使用 "N/A"
        std::string modelNameStr = modelNameOpt.value_or("N/A");

        // --- 使用获取到的 modelNameStr 进行日志记录 ---
        ignition::common::Console::dbg << "Configure called for model: " << modelNameStr << std::endl;

        // --- 1. Parse SDF ---
        if (!ParseSDF(_sdf))
        {
            ignition::common::Console::err << "Failed to parse SDF for HectorGazeboFortressPlugin for model: " << modelNameStr << "." << std::endl;
            return;
        }
        this->sdfParsed_ = true;
        ignition::common::Console::msg << "SDF parsed successfully for " << modelNameStr << "." << std::endl;

        // --- 2. Initialize ROS ---
        InitROS();
        if (!this->rosInitialized_)
        {
            ignition::common::Console::err << "Failed to initialize ROS components for " << modelNameStr << "." << std::endl;
            return;
        }
        ignition::common::Console::msg << "ROS components initialized for " << modelNameStr << "." << std::endl;

        // --- 3. Initialize Ignition Transport ---
        InitIgnitionTransport();
        if (!this->ignTransportInitialized_)
        {
            ignition::common::Console::err << "Failed to initialize Ignition Transport subscribers for " << modelNameStr << "." << std::endl;
            return;
        }
        ignition::common::Console::msg << "Ignition Transport initialized for " << modelNameStr << "." << std::endl;
    }


    bool HectorGazeboFortressPlugin::ParseSDF(const std::shared_ptr<const sdf::Element> &_sdf)
    {
        // --- Joint Names ---
        if (!_sdf->HasElement("joint_name")) {
            ignition::common::Console::err << "Missing required <joint_name> elements in SDF." << std::endl;
            return false;
        }

        sdf::ElementPtr jointElem = _sdf->GetElementImpl("joint_name");

        while (jointElem) {
            jointNames_.push_back(jointElem->Get<std::string>());
            ignition::common::Console::dbg << "Found joint to control: " << jointNames_.back() << std::endl;
            jointElem = jointElem->GetNextElement("joint_name");
        }
        if (jointNames_.empty()) {
            ignition::common::Console::err << "No joint names specified via <joint_name> tags." << std::endl;
            return false;
        }
        numJoints_ = jointNames_.size();
        ignition::common::Console::msg << "Expecting " << numJoints_ << " joints based on SDF." << std::endl;

        // --- Link Name ---
        baseLinkName_ = _sdf->Get<std::string>("base_link_name", "base_link").first;
        ignition::common::Console::msg << "Using base link name: " << baseLinkName_ << std::endl;

        // --- ROS Topics ---
        rosCmdTopic_ = _sdf->Get<std::string>("ros_cmd_topic", "/Hector_Command").first;
        rosRobotStateTopic_ = _sdf->Get<std::string>("ros_robot_state_topic", "/Hector_State").first;
        rosContactStateTopic_ = _sdf->Get<std::string>("ros_contact_state_topic", "/true_toe_floor_contact").first;
        ignition::common::Console::msg << "ROS Command Topic: " << rosCmdTopic_ << std::endl;
        ignition::common::Console::msg << "ROS Robot State Topic: " << rosRobotStateTopic_ << std::endl;
        ignition::common::Console::msg << "ROS Contact State Topic: " << rosContactStateTopic_ << std::endl;

        // --- Ignition Topics ---
        if (!_sdf->HasElement("ign_imu_topic")) {
            ignition::common::Console::err << "Missing required <ign_imu_topic> element in SDF." << std::endl;
            return false;
        }
        ignImuTopic_ = _sdf->Get<std::string>("ign_imu_topic");

        if (!_sdf->HasElement("ign_left_contact_topic")) {
            ignition::common::Console::err << "Missing required <ign_left_contact_topic> element in SDF." << std::endl;
            return false;
        }
        ignLeftContactTopic_ = _sdf->Get<std::string>("ign_left_contact_topic");

        if (!_sdf->HasElement("ign_right_contact_topic")) {
            ignition::common::Console::err << "Missing required <ign_right_contact_topic> element in SDF." << std::endl;
            return false;
        }
        ignRightContactTopic_ = _sdf->Get<std::string>("ign_right_contact_topic");

        ignition::common::Console::msg << "Ignition IMU Topic: " << ignImuTopic_ << std::endl;
        ignition::common::Console::msg << "Ignition Left Contact Topic: " << ignLeftContactTopic_ << std::endl;
        ignition::common::Console::msg << "Ignition Right Contact Topic: " << ignRightContactTopic_ << std::endl;

        // --- Ground Collision Name ---
        groundCollisionName_ = _sdf->Get<std::string>("ground_collision_name", "ground_plane::link::collision").first;
        ignition::common::Console::msg << "Using ground collision name pattern: '" << groundCollisionName_ << "'" << std::endl;

        return true;
    }

    void HectorGazeboFortressPlugin::InitROS()
    {
        if (!rclcpp::ok()) {
            ignition::common::Console::err << "RCLCPP not initialized when trying to create node. Aborting ROS init." << std::endl;
            return;
        }
        std::string node_name = "hector_ign_plugin_" + std::to_string(this->modelEntity_);
        node_name.erase(std::remove_if(node_name.begin(), node_name.end(),
                                      [](char c) { return !std::isalnum(c) && c != '_'; }),
                       node_name.end());

        try {
            rosNode_ = std::make_shared<rclcpp::Node>(node_name);
        } catch (const std::exception &e) {
            ignition::common::Console::err << "Failed to create ROS 2 node '" << node_name << "': " << e.what() << std::endl;
            return;
        }

        if (!rosNode_) {
            ignition::common::Console::err << "Failed to create ROS 2 node pointer for '" << node_name << "'" << std::endl;
            return;
        }

        cmdMutex_.lock();
        lastRosCmd_.reset();
        cmdMutex_.unlock();

        rclcpp::QoS qos_profile_sub(rclcpp::KeepLast(10));
        try {
            rosCmdSub_ = rosNode_->create_subscription<laser_interfaces::msg::RobotCommand>(
                rosCmdTopic_, qos_profile_sub,
                std::bind(&HectorGazeboFortressPlugin::RosCommandCallback, this, std::placeholders::_1));
        } catch (const std::exception &e) {
            ignition::common::Console::err << "Failed to create ROS subscription to '" << rosCmdTopic_ << "': " << e.what() << std::endl;
            rosNode_.reset();
            return;
        }
        if (!rosCmdSub_) {
            ignition::common::Console::err << "ROS subscription pointer is null after creation for topic '" << rosCmdTopic_ << "'" << std::endl;
            rosNode_.reset();
            return;
        }

        rclcpp::QoS qos_profile_pub(rclcpp::KeepLast(10));
        try {
            rosRobotStatePub_ = rosNode_->create_publisher<laser_interfaces::msg::RobotState>(
                rosRobotStateTopic_, qos_profile_pub);
        } catch (const std::exception &e) {
            ignition::common::Console::err << "Failed to create ROS publisher for '" << rosRobotStateTopic_ << "': " << e.what() << std::endl;
            rosNode_.reset();
            return;
        }
        if (!rosRobotStatePub_) {
            ignition::common::Console::err << "ROS publisher pointer is null after creation for topic '" << rosRobotStateTopic_ << "'" << std::endl;
            rosNode_.reset();
            return;
        }

        try {
            rosContactStatePub_ = rosNode_->create_publisher<laser_interfaces::msg::ContactState>(
                rosContactStateTopic_, qos_profile_pub);
        } catch (const std::exception &e) {
            ignition::common::Console::err << "Failed to create ROS publisher for '" << rosContactStateTopic_ << "': " << e.what() << std::endl;
            rosNode_.reset();
            return;
        }
        if (!rosContactStatePub_) {
            ignition::common::Console::err << "ROS publisher pointer is null after creation for topic '" << rosContactStateTopic_ << "'" << std::endl;
            rosNode_.reset();
            return;
        }

        this->rosInitialized_ = true;
        // Corrected Logging Syntax (lowercase):
        ignition::common::Console::msg << "ROS Node '" << rosNode_->get_name() << "' initialized successfully." << std::endl;
    }

    void HectorGazeboFortressPlugin::InitIgnitionTransport()
    {
        if (!ignNode_.Subscribe(ignImuTopic_, &HectorGazeboFortressPlugin::IgnImuCallback, this))
        {
            ignition::common::Console::err << "Error subscribing to Ignition IMU topic [" << ignImuTopic_ << "]" << std::endl;
            return;
        }
        if (!ignNode_.Subscribe(ignLeftContactTopic_, &HectorGazeboFortressPlugin::IgnLeftContactCallback, this))
        {
            ignition::common::Console::err << "Error subscribing to Ignition Left Contact topic [" << ignLeftContactTopic_ << "]" << std::endl;
            return;
        }
        if (!ignNode_.Subscribe(ignRightContactTopic_, &HectorGazeboFortressPlugin::IgnRightContactCallback, this))
        {
            ignition::common::Console::err << "Error subscribing to Ignition Right Contact topic [" << ignRightContactTopic_ << "]" << std::endl;
            return;
        }
        this->ignTransportInitialized_ = true;
        ignition::common::Console::msg << "Ignition Transport subscriptions initialized." << std::endl;
    }

    bool HectorGazeboFortressPlugin::FindEntities(ignition::gazebo::EntityComponentManager &_ecm)
    {
        ignition::gazebo::Model model(this->modelEntity_);

        this->baseLinkEntity_ = model.LinkByName(_ecm, this->baseLinkName_);
        if (this->baseLinkEntity_ == ignition::gazebo::kNullEntity) {
            ignition::common::Console::err << "Base link named '" << this->baseLinkName_ << "' not found within model." << std::endl;
            return false;
        }
        ignition::common::Console::msg << "Found base link '" << this->baseLinkName_ << "' with entity ID: " << this->baseLinkEntity_ << std::endl;

        // Use lowercase 'warn' for warnings
        if (!_ecm.Component<ignition::gazebo::components::WorldPose>(this->baseLinkEntity_)) {
            ignition::common::Console::warn << "WorldPose component not found for base link '" << this->baseLinkName_ << "'. State reading might be incomplete." << std::endl;
        }
        if (!_ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->baseLinkEntity_)) {
            ignition::common::Console::warn << "WorldLinearVelocity component not found for base link '" << this->baseLinkName_ << "'. State reading might be incomplete." << std::endl;
        }
        if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(this->baseLinkEntity_)) {
            ignition::common::Console::warn << "WorldAngularVelocity component not found for base link '" << this->baseLinkName_ << "'. State reading might be incomplete." << std::endl;
        }

        this->jointEntities_.clear();
        this->jointEntities_.reserve(this->numJoints_);
        bool all_joints_found = true;
        for (const auto& name : this->jointNames_) {
            ignition::gazebo::Entity jointEntity = model.JointByName(_ecm, name);
            if (jointEntity == ignition::gazebo::kNullEntity) {
                ignition::common::Console::err << "Joint named '" << name << "' not found within model." << std::endl;
                all_joints_found = false;
                continue;
            }
            this->jointEntities_.push_back(jointEntity);
            ignition::common::Console::msg << "Found joint '" << name << "' with entity ID: " << jointEntity << std::endl;

            if (!_ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity)) {
                _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointForceCmd({0.0}));
                ignition::common::Console::msg << "Created JointForceCmd component for joint '" << name << "'" << std::endl;
            }
            // Use lowercase 'warn'
            if (!_ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity)) {
                ignition::common::Console::warn << "JointPosition component not found for joint '" << name << "'. State reading might fail." << std::endl;
            }
            if (!_ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity)) {
                ignition::common::Console::warn << "JointVelocity component not found for joint '" << name << "'. State reading might fail." << std::endl;
            }
        }

        if (!all_joints_found || this->jointEntities_.size() != this->numJoints_) {
            ignition::common::Console::err << "Failed to find all " << this->numJoints_ << " joints specified in SDF. Found " << this->jointEntities_.size() << "." << std::endl;
            this->jointEntities_.clear();
            return false;
        }

        this->entitiesFound_ = true;
        ignition::common::Console::msg << "All required entities (base link and " << this->jointEntities_.size() << " joints) found." << std::endl;
        return true;
    }

    void HectorGazeboFortressPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                               ignition::gazebo::EntityComponentManager &_ecm)
    {
        if (!this->sdfParsed_ || !this->rosInitialized_ || !this->ignTransportInitialized_) {
            return;
        }

        if (!this->entitiesFound_) {
            if (!FindEntities(_ecm)) {
                ignition::common::Console::err << "Failed to find all required entities during the first PreUpdate. Plugin will not execute further." << std::endl;
                return;
            }
        }

        if (this->rosNode_) {
            rclcpp::spin_some(this->rosNode_);
        } else {
            // Log error if ROS node becomes invalid after initialization?
            // ignition::common::Console::err << "ROS Node pointer invalid during PreUpdate." << std::endl;
            return;
        }

        if (_info.paused) {
            return;
        }

        ApplyControl(_ecm);
        ReadAndPublishState(_info, _ecm);
    }

    void HectorGazeboFortressPlugin::ApplyControl(ignition::gazebo::EntityComponentManager &_ecm)
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        const double test_torque = 0.0005;
        if (!lastRosCmd_.has_value()) {
            for (size_t i = 0; i < jointEntities_.size(); ++i) {
                auto forceComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntities_[i]);
                if (forceComp) {
                    if (forceComp->Data().empty()) forceComp->Data().resize(1);
                    forceComp->Data()[0] = test_torque;
                    _ecm.SetChanged(jointEntities_[i], ignition::gazebo::components::JointForceCmd::typeId);
                }
            }
            return;
        }

        const auto& current_cmd = lastRosCmd_.value();
        if (current_cmd.motor_command.size() != jointEntities_.size()) {
            // Use lowercase 'warn'
            ignition::common::Console::warn << "Received RobotCommand with " << current_cmd.motor_command.size()
                   << " motor entries, but plugin controls " << jointEntities_.size()
                   << " joints. Skipping command application and applying zero torque." << std::endl;
            for (size_t i = 0; i < jointEntities_.size(); ++i) {
                auto forceComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntities_[i]);
                if (forceComp) {
                    if (forceComp->Data().empty()) forceComp->Data().resize(1);
                    forceComp->Data()[0] = 0.0;
                    _ecm.SetChanged(jointEntities_[i], ignition::gazebo::components::JointForceCmd::typeId);
                }
            }
            lastRosCmd_.reset();
            return;
        }

        for (size_t i = 0; i < jointEntities_.size(); ++i)
        {
            ignition::gazebo::Entity jointEntity = jointEntities_[i];
            double current_pos = 0.0;
            double current_vel = 0.0;

            auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
            if (posComp && !posComp->Data().empty()) {
                current_pos = posComp->Data()[0];
            } // Else: Assume 0.0, warning logged in FindEntities if component missing

            auto velComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);
            if (velComp && !velComp->Data().empty()) {
                current_vel = velComp->Data()[0];
            } // Else: Assume 0.0, warning logged in FindEntities if component missing

            const auto& motor_cmd = current_cmd.motor_command[i];
            double target_pos = motor_cmd.q;
            double target_vel = motor_cmd.dq;
            double kp = motor_cmd.kp;
            double kd = motor_cmd.kd;
            double tau_ff = motor_cmd.tau;

            double pos_error = target_pos - current_pos;
            double vel_error = target_vel - current_vel;
            double torque_cmd = (kp * pos_error) + (kd * vel_error) + tau_ff;

            auto forceComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity);
            if (forceComp) {
                if (forceComp->Data().empty()) forceComp->Data().resize(1);
                forceComp->Data()[0] = torque_cmd;
                _ecm.SetChanged(jointEntity, ignition::gazebo::components::JointForceCmd::typeId);
            } else {
                ignition::common::Console::err << "JointForceCmd component missing for joint " << jointNames_[i] << " during ApplyControl! Cannot apply torque." << std::endl;
            }
        }
        // Optional: Reset command after applying
        // lastRosCmd_.reset();
    }

    void HectorGazeboFortressPlugin::ReadAndPublishState(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm)
    {
        // --- 1. Populate RobotState Message ---
        if (this->baseLinkEntity_ != ignition::gazebo::kNullEntity) {
            auto poseComp = _ecm.Component<ignition::gazebo::components::WorldPose>(this->baseLinkEntity_);
            auto linVelComp = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->baseLinkEntity_);
            auto angVelComp = _ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(this->baseLinkEntity_);

            if (poseComp) {
                const ignition::math::Pose3d& pose = poseComp->Data();
                if (robotStateMsg_.body_position.size() == 3) {
                    robotStateMsg_.body_position[0] = static_cast<float>(pose.Pos().X());
                    robotStateMsg_.body_position[1] = static_cast<float>(pose.Pos().Y());
                    robotStateMsg_.body_position[2] = static_cast<float>(pose.Pos().Z());
                }
                if (!imuReceived_ && robotStateMsg_.imu.size() > 0 && robotStateMsg_.imu[0].quaternion.size() == 4) {
                    robotStateMsg_.imu[0].quaternion[0] = static_cast<float>(pose.Rot().X());
                    robotStateMsg_.imu[0].quaternion[1] = static_cast<float>(pose.Rot().Y());
                    robotStateMsg_.imu[0].quaternion[2] = static_cast<float>(pose.Rot().Z());
                    robotStateMsg_.imu[0].quaternion[3] = static_cast<float>(pose.Rot().W());
                }
            } else {
                std::fill(robotStateMsg_.body_position.begin(), robotStateMsg_.body_position.end(), 0.0f);
                // Warning logged in FindEntities
            }

            if (linVelComp) {
                const ignition::math::Vector3d& lin_vel = linVelComp->Data();
                if (robotStateMsg_.body_velocity.size() == 3) {
                    robotStateMsg_.body_velocity[0] = static_cast<float>(lin_vel.X());
                    robotStateMsg_.body_velocity[1] = static_cast<float>(lin_vel.Y());
                    robotStateMsg_.body_velocity[2] = static_cast<float>(lin_vel.Z());
                }
            } else {
                std::fill(robotStateMsg_.body_velocity.begin(), robotStateMsg_.body_velocity.end(), 0.0f);
                // Warning logged in FindEntities
            }

            if (angVelComp && !imuReceived_ && robotStateMsg_.imu.size() > 0 && robotStateMsg_.imu[0].gyroscope.size() == 3) {
                const ignition::math::Vector3d& ang_vel = angVelComp->Data();
                robotStateMsg_.imu[0].gyroscope[0] = static_cast<float>(ang_vel.X());
                robotStateMsg_.imu[0].gyroscope[1] = static_cast<float>(ang_vel.Y());
                robotStateMsg_.imu[0].gyroscope[2] = static_cast<float>(ang_vel.Z());
            }
            else if (!imuReceived_ && robotStateMsg_.imu.size() > 0) {
                std::fill(robotStateMsg_.imu[0].gyroscope.begin(), robotStateMsg_.imu[0].gyroscope.end(), 0.0f);
                // Warning logged in FindEntities
            }
        } else {
            // Base link entity is null, zero out fields
            std::fill(robotStateMsg_.body_position.begin(), robotStateMsg_.body_position.end(), 0.0f);
            std::fill(robotStateMsg_.body_velocity.begin(), robotStateMsg_.body_velocity.end(), 0.0f);
            if (robotStateMsg_.imu.size() > 0) {
                std::fill(robotStateMsg_.imu[0].quaternion.begin(), robotStateMsg_.imu[0].quaternion.end(), 0.0f);
                std::fill(robotStateMsg_.imu[0].gyroscope.begin(), robotStateMsg_.imu[0].gyroscope.end(), 0.0f);
                std::fill(robotStateMsg_.imu[0].accelerometer.begin(), robotStateMsg_.imu[0].accelerometer.end(), 0.0f);
            }
        }

        { // IMU Mutex Scope
            std::lock_guard<std::mutex> lock(imuMutex_);
            if (imuReceived_ && robotStateMsg_.imu.size() > 0) {
                if (lastIgnImuMsg_.has_orientation()) {
                    if (robotStateMsg_.imu[0].quaternion.size() == 4) {
                        robotStateMsg_.imu[0].quaternion[0] = static_cast<float>(lastIgnImuMsg_.orientation().x());
                        robotStateMsg_.imu[0].quaternion[1] = static_cast<float>(lastIgnImuMsg_.orientation().y());
                        robotStateMsg_.imu[0].quaternion[2] = static_cast<float>(lastIgnImuMsg_.orientation().z());
                        robotStateMsg_.imu[0].quaternion[3] = static_cast<float>(lastIgnImuMsg_.orientation().w());
                    }
                }
                if (lastIgnImuMsg_.has_angular_velocity()) {
                    if (robotStateMsg_.imu[0].gyroscope.size() == 3) {
                        robotStateMsg_.imu[0].gyroscope[0] = static_cast<float>(lastIgnImuMsg_.angular_velocity().x());
                        robotStateMsg_.imu[0].gyroscope[1] = static_cast<float>(lastIgnImuMsg_.angular_velocity().y());
                        robotStateMsg_.imu[0].gyroscope[2] = static_cast<float>(lastIgnImuMsg_.angular_velocity().z());
                    }
                }
                if (lastIgnImuMsg_.has_linear_acceleration()) {
                    if (robotStateMsg_.imu[0].accelerometer.size() == 3) {
                        robotStateMsg_.imu[0].accelerometer[0] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().x());
                        robotStateMsg_.imu[0].accelerometer[1] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().y());
                        robotStateMsg_.imu[0].accelerometer[2] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().z());
                    }
                }
            } else if (robotStateMsg_.imu.size() > 0) {
                // Zero out accel if no IMU data (gyro/orientation handled above)
                std::fill(robotStateMsg_.imu[0].accelerometer.begin(), robotStateMsg_.imu[0].accelerometer.end(), 0.0f);
            }
        } // End IMU Mutex Scope

        // --- Get Joint States ---
        if (robotStateMsg_.motor_state.size() == jointEntities_.size()) {
            for (size_t i = 0; i < jointEntities_.size(); ++i) {
                ignition::gazebo::Entity jointEntity = jointEntities_[i];
                auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
                auto velComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);
                auto forceCmdComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity);

                if (posComp && !posComp->Data().empty()) {
                    robotStateMsg_.motor_state[i].q = static_cast<float>(posComp->Data()[0]);
                } else {
                    robotStateMsg_.motor_state[i].q = 0.0f;
                }
                if (velComp && !velComp->Data().empty()) {
                    robotStateMsg_.motor_state[i].dq = static_cast<float>(velComp->Data()[0]);
                } else {
                    robotStateMsg_.motor_state[i].dq = 0.0f;
                }
                if (forceCmdComp && !forceCmdComp->Data().empty()) {
                    robotStateMsg_.motor_state[i].tauest = static_cast<float>(forceCmdComp->Data()[0]);
                } else {
                    robotStateMsg_.motor_state[i].tauest = 0.0f;
                }
            }
        } else {
            ignition::common::Console::err << "Mismatch between robotStateMsg_.motor_state size ("
                      << robotStateMsg_.motor_state.size() << ") and found joint entities ("
                      << jointEntities_.size() << "). Skipping joint state population." << std::endl;
        }

        // --- 2. Populate ContactState Message ---
        { // Contact Mutex Scope
            std::lock_guard<std::mutex> lock(contactMutex_);
            if (contactStateMsg_.contact_state.size() == 2) {
                contactStateMsg_.contact_state[0] = leftContact_ ? 1.0f : 0.0f;
                contactStateMsg_.contact_state[1] = rightContact_ ? 1.0f : 0.0f;
            }
            // Reset flags for next step detection
            leftContact_ = false;
            rightContact_ = false;
        } // End Contact Mutex Scope

        // --- 3. Publish ROS Messages ---
        if (rosRobotStatePub_) {
            rosRobotStatePub_->publish(robotStateMsg_);
        }
        if (rosContactStatePub_) {
            rosContactStatePub_->publish(contactStateMsg_);
        }
    }

    // --- Callbacks ---

    void HectorGazeboFortressPlugin::RosCommandCallback(const laser_interfaces::msg::RobotCommand::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        lastRosCmd_ = *_msg;
        // ignition::common::Console::dbg << "Received new ROS command." << std::endl; // Optional
    }

    void HectorGazeboFortressPlugin::IgnImuCallback(const ignition::msgs::IMU &_msg)
    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        lastIgnImuMsg_ = _msg;
        imuReceived_ = true;
        // ignition::common::Console::dbg << "Received Ignition IMU data." << std::endl; // Optional
    }

    void HectorGazeboFortressPlugin::IgnLeftContactCallback(const ignition::msgs::Contacts &_msg)
    {
        bool contact_with_ground = false;
        for (int i = 0; i < _msg.contact_size(); ++i) {
            const auto& contact = _msg.contact(i);
            if (contact.collision1().name().find(this->groundCollisionName_) != std::string::npos ||
                contact.collision2().name().find(this->groundCollisionName_) != std::string::npos)
            {
                contact_with_ground = true;
                break;
            }
        }
        if (contact_with_ground) {
            std::lock_guard<std::mutex> lock(contactMutex_);
            leftContact_ = true;
            // ignition::common::Console::dbg << "Left Contact detected." << std::endl; // Optional
        }
    }

    void HectorGazeboFortressPlugin::IgnRightContactCallback(const ignition::msgs::Contacts &_msg)
    {
        bool contact_with_ground = false;
        for (int i = 0; i < _msg.contact_size(); ++i) {
            const auto& contact = _msg.contact(i);
            if (contact.collision1().name().find(this->groundCollisionName_) != std::string::npos ||
               contact.collision2().name().find(this->groundCollisionName_) != std::string::npos)
            {
                contact_with_ground = true;
                break;
            }
        }
        if (contact_with_ground) {
            std::lock_guard<std::mutex> lock(contactMutex_);
            rightContact_ = true;
            // ignition::common::Console::dbg << "Right Contact detected." << std::endl; // Optional
        }
    }
}

// --- Plugin Registration ---
IGNITION_ADD_PLUGIN(hector_gazebo_plugins::HectorGazeboFortressPlugin,
              ignition::gazebo::System,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemConfigure,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemPreUpdate)
