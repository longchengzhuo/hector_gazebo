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
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

// *** NECESSARY FOR LOGGING ***
#include <ignition/common/Console.hh>

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
        jointForces_.clear();
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
        ignition::common::Console::err << "-------HECTORv2-GAZEBO-----------S---------T---------A---------R---------T-------HECTORv2-GAZEBO-----------" << std::endl;


        this->modelEntity_ = _entity;
        ignition::gazebo::Model model(this->modelEntity_);
        std::optional<std::string> modelNameOpt = model.Name(_ecm);
        std::string modelNameStr = "N/A"; // Default value
        if (modelNameOpt.has_value()) { // Check if the optional contains a value
            modelNameStr = modelNameOpt.value(); // Get the value if it exists
        }
        ignition::common::Console::warn << "Configure called for model: " << modelNameStr << std::endl;

        // --- 1. Parse SDF ---
        if (!ParseSDF(_sdf))
        {
            ignition::common::Console::err << "Failed to parse SDF for HectorGazeboFortressPlugin for model: " << modelNameStr << "." << std::endl;
            return;
        }
        this->sdfParsed_ = true;
        ignition::common::Console::warn << "SDF parsed successfully for " << modelNameStr << "." << std::endl;

        // --- 2. Initialize ROS ---
        InitROS();
        if (!this->rosInitialized_)
        {
            ignition::common::Console::err << "Failed to initialize ROS components for " << modelNameStr << "." << std::endl;
            return;
        }
        ignition::common::Console::warn << "ROS components initialized for " << modelNameStr << "." << std::endl;

        // --- 3. Initialize Ignition Transport ---
        InitIgnitionTransport();
        if (!this->ignTransportInitialized_)
        {
            ignition::common::Console::err << "Failed to initialize Ignition Transport subscribers for " << modelNameStr << "." << std::endl;
            return;
        }
        ignition::common::Console::warn << "Ignition Transport initialized for " << modelNameStr << "." << std::endl;
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
            ignition::common::Console::warn << "Found joint to control: " << jointNames_.back() << std::endl;
            jointElem = jointElem->GetNextElement("joint_name");
        }
        if (jointNames_.empty()) {
            ignition::common::Console::err << "No joint names specified via <joint_name> tags." << std::endl;
            return false;
        }
        numJoints_ = jointNames_.size();
        ignition::common::Console::warn << "Expecting " << numJoints_ << " joints based on SDF." << std::endl;

        // --- Link Name ---
        baseLinkName_ = _sdf->Get<std::string>("base_link_name", "base_link").first;
        ignition::common::Console::warn << "Using base link name: " << baseLinkName_ << std::endl;

        // --- ROS Topics ---
        rosCmdTopic_ = _sdf->Get<std::string>("ros_cmd_topic", "/Hector_Command").first;
        rosRobotStateTopic_ = _sdf->Get<std::string>("ros_robot_state_topic", "/Hector_State").first;
        rosContactStateTopic_ = _sdf->Get<std::string>("ros_contact_state_topic", "/true_toe_floor_contact").first;
        ignition::common::Console::warn << "ROS Command Topic: " << rosCmdTopic_ << std::endl;
        ignition::common::Console::warn << "ROS Robot State Topic: " << rosRobotStateTopic_ << std::endl;
        ignition::common::Console::warn << "ROS Contact State Topic: " << rosContactStateTopic_ << std::endl;

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

        ignition::common::Console::warn << "Ignition IMU Topic: " << ignImuTopic_ << std::endl;
        ignition::common::Console::warn << "Ignition Left Contact Topic: " << ignLeftContactTopic_ << std::endl;
        ignition::common::Console::warn << "Ignition Right Contact Topic: " << ignRightContactTopic_ << std::endl;

        // --- Ground Collision Name ---
        groundCollisionName_ = _sdf->Get<std::string>("ground_collision_name", "ground_plane::link::collision").first;
        ignition::common::Console::warn << "Using ground collision name pattern: '" << groundCollisionName_ << "'" << std::endl;

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
        ignition::common::Console::warn << "ROS Node '" << rosNode_->get_name() << "' initialized successfully." << std::endl;
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
        ignition::common::Console::warn << "Ignition Transport subscriptions initialized." << std::endl;
    }

    bool HectorGazeboFortressPlugin::FindEntities(ignition::gazebo::EntityComponentManager &_ecm)
    {
        ignition::gazebo::Model model(this->modelEntity_);

        this->baseLinkEntity_ = model.LinkByName(_ecm, this->baseLinkName_);
        if (this->baseLinkEntity_ == ignition::gazebo::kNullEntity) {
            ignition::common::Console::err << "Base link named '" << this->baseLinkName_ << "' not found within model." << std::endl;
            return false;
        }
        ignition::common::Console::warn << "Found base link '" << this->baseLinkName_ << "' with entity ID: " << this->baseLinkEntity_ << std::endl;


        if (!_ecm.HasEntity(this->baseLinkEntity_))
        {
             ignition::common::Console::err << "Base link entity " << this->baseLinkEntity_ << " seems invalid." << std::endl;
             return false;
        }
        // Check for components needed in PostUpdate (reading state)
        if (!_ecm.Component<ignition::gazebo::components::WorldPose>(this->baseLinkEntity_)) {
            ignition::common::Console::warn << "Creating WorldPose component for base link '" << this->baseLinkName_ << "' (might indicate physics system issue if missing)." << std::endl;
            _ecm.CreateComponent(this->baseLinkEntity_, ignition::gazebo::components::WorldPose());
        }
        if (!_ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->baseLinkEntity_)) {
            ignition::common::Console::warn << "Creating WorldLinearVelocity component for base link '" << this->baseLinkName_ << "'." << std::endl;
             _ecm.CreateComponent(this->baseLinkEntity_, ignition::gazebo::components::WorldLinearVelocity());
        }
        if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(this->baseLinkEntity_)) {
             ignition::common::Console::warn << "Creating WorldAngularVelocity component for base link '" << this->baseLinkName_ << "'." << std::endl;
             _ecm.CreateComponent(this->baseLinkEntity_, ignition::gazebo::components::WorldAngularVelocity());
        }


        this->jointEntities_.clear();
        this->jointEntities_.reserve(this->numJoints_);
        bool all_joints_found = true;
        for (const auto& name : this->jointNames_) {
            ignition::gazebo::Entity jointEntity = model.JointByName(_ecm, name);
            if (jointEntity == ignition::gazebo::kNullEntity) {
                ignition::common::Console::err << "Joint named '" << name << "' not found within model." << std::endl;
                all_joints_found = false;
                continue; // Skip this joint
            }
            if (!_ecm.HasEntity(jointEntity))
            {
                ignition::common::Console::err << "Joint entity " << jointEntity << " for name '"<< name << "' seems invalid." << std::endl;
                 all_joints_found = false;
                 continue;
            }

            this->jointEntities_.push_back(jointEntity);
            ignition::common::Console::warn << "Found joint '" << name << "' with entity ID: " << jointEntity << std::endl;

            // Check for components needed in PreUpdate (applying control)
            if (!_ecm.Component<ignition::gazebo::components::JointForce>(jointEntity)) {
                _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointForce({0.0}));
                ignition::common::Console::warn << "Created JointForce component for joint '" << name << "'" << std::endl;
            }
            if (!_ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity)) {
                _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointForceCmd({0.0}));
                ignition::common::Console::warn << "Created JointForceCmd component for joint '" << name << "'" << std::endl;
            }
            // Check for components needed for calculations within ApplyControl (reading state *before* command)
            if (!_ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity)) {
                 ignition::common::Console::warn << "Creating JointPosition component for joint '" << name << "'. State reading might fail initially." << std::endl;
                 // Optionally create with a default value, though physics system should populate it
                 _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointPosition({0.0}));
            }
            if (!_ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity)) {
                 ignition::common::Console::warn << "Creating JointVelocity component for joint '" << name << "'. State reading might fail initially." << std::endl;
                 _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointVelocity({0.0}));
            }
        }

        if (!all_joints_found || this->jointEntities_.size() != this->numJoints_) {
            ignition::common::Console::err << "Failed to find all " << this->numJoints_ << " joints specified in SDF. Found " << this->jointEntities_.size() << "." << std::endl;
            this->jointEntities_.clear(); // Ensure consistency
            return false;
        }

        this->jointForces_.resize(this->jointEntities_.size());
        for (auto &force : this->jointForces_) {
            force.resize(1, 0.0);  // 默认每个关节只有一个力分量
        }
        this->entitiesFound_ = true;
        ignition::common::Console::warn << "All required entities (base link and " << this->jointEntities_.size() << " joints) found." << std::endl;
        return true;
    }

    // --- PreUpdate ---
    void HectorGazeboFortressPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                               ignition::gazebo::EntityComponentManager &_ecm)
    {
        // Check initialization status
        if (!this->sdfParsed_ || !this->rosInitialized_ || !this->ignTransportInitialized_) {
            return; // Not configured yet
        }

        // Find entities on the first valid update if not found yet
        if (!this->entitiesFound_) {
            if (!FindEntities(_ecm)) {
                ignition::common::Console::err << "Failed to find all required entities during PreUpdate. Plugin will not execute." << std::endl;
                return;
            }
        }

        // Spin ROS node to process incoming messages (like commands)
        if (this->rosNode_) {
            // Use non-blocking spin_some
            rclcpp::spin_some(this->rosNode_);
        } else {
             if (this->rosInitialized_) { // Only log error if it was initialized before
                ignition::common::Console::err << "ROS Node pointer invalid during PreUpdate." << std::endl;
             }
            return; // Cannot process commands or publish state
        }

        // Do nothing if paused
        if (_info.paused) {
            return;
        }

        // --- Apply Control ---
        // Apply forces/torques based on the latest command received
        ApplyControl(_ecm);
    }

    void HectorGazeboFortressPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                                           ignition::gazebo::EntityComponentManager &_ecm)
    {
        // 检查初始化状态
        if (!this->sdfParsed_ || !this->rosInitialized_ || !this->ignTransportInitialized_ || !this->entitiesFound_) {
            return; // 未准备好或实体缺失
        }

        // 如果暂停则不执行
        if (_info.paused) {
            return;
        }

    }

    // --- PostUpdate ---
    void HectorGazeboFortressPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                                const ignition::gazebo::EntityComponentManager &_ecm)
    {
         // Check initialization and entity status (read-only check)
        if (!this->sdfParsed_ || !this->rosInitialized_ || !this->ignTransportInitialized_ || !this->entitiesFound_) {
            return; // Not ready or entities missing
        }

        // Do nothing if paused
        if (_info.paused) {
            return;
        }

         ReadAndPublishState(_info, _ecm);
    }


    void HectorGazeboFortressPlugin::ApplyControl(ignition::gazebo::EntityComponentManager &_ecm)
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        const double no_cmd_torque = 0.00;

        // Check if we have a valid command
        if (!lastRosCmd_.has_value()) {
            // ignition::common::Console::err << "No ROS command received yet. Creating default command." << std::endl;

            lastRosCmd_.emplace(); // 调用默认构造函数

            if (lastRosCmd_.has_value()) {
                for (size_t i = 0; i < jointEntities_.size(); ++i) {
                    lastRosCmd_->motor_command[i].q = 1.0;    // 默认目标位置
                    lastRosCmd_->motor_command[i].dq = 0.0;   // 默认目标速度
                    lastRosCmd_->motor_command[i].kp = 1.0;   // 默认 Kp (可能需要一个小的阻尼?)
                    lastRosCmd_->motor_command[i].kd = 1.0;   // 默认 Kd (提供一些阻尼)
                    lastRosCmd_->motor_command[i].tau = 0.0;   // 默认前馈力矩
                }
                ignition::common::Console::err << "----------------Applied default values to newly created command.-------------------" << std::endl;
            } else {
                ignition::common::Console::err << "Failed to emplace a default RobotCommand!" << std::endl;
                for (size_t i = 0; i < jointEntities_.size(); ++i) {
                    _ecm.SetComponentData<ignition::gazebo::components::JointForceCmd>(jointEntities_[i], {no_cmd_torque});
                }
                return;
            }
        }

        const auto& current_cmd = lastRosCmd_.value();

        // Verify command size matches number of joints
        if (current_cmd.motor_command.size() != jointEntities_.size()) {
            ignition::common::Console::err << "Received RobotCommand with " << current_cmd.motor_command.size()
                   << " motor entries, but plugin controls " << jointEntities_.size()
                   << " joints. Applying zero torque." << std::endl;
            // Apply zero torque due to mismatch
            for (size_t i = 0; i < jointEntities_.size(); ++i) {
                 _ecm.SetComponentData<ignition::gazebo::components::JointForceCmd>(
                    jointEntities_[i], {no_cmd_torque});
            }
            return;
        }

        // Apply the command from the message
        for (size_t i = 0; i < jointEntities_.size(); ++i)
        {
            //ignition::common::Console::err << "----------control--------" << std::endl;

            ignition::gazebo::Entity jointEntity = jointEntities_[i];
            double current_pos = 0.0;
            double current_vel = 0.0;


            auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
            if (posComp && !posComp->Data().empty()) {
                current_pos = posComp->Data()[0];
                 //ignition::common::Console::warn << "joint " << jointNames_[i] <<" current_pos: "<< current_pos << std::endl; // Debug
            } else {
                 ignition::common::Console::err << "JointPosition missing/empty for " << jointNames_[i] << " in ApplyControl." << std::endl;
            }

            auto velComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);
            if (velComp && !velComp->Data().empty()) {
                current_vel = velComp->Data()[0];
                 //ignition::common::Console::warn << "joint " << jointNames_[i] <<" current_vel: "<< current_vel << std::endl; // Debug
            } else {
                 ignition::common::Console::err << "JointVelocity missing/empty for " << jointNames_[i] << " in ApplyControl." << std::endl;
            }

            const auto& motor_cmd = current_cmd.motor_command[i];
            double target_pos = motor_cmd.q;
            double target_vel = motor_cmd.dq;
            double kp = motor_cmd.kp;
            double kd = motor_cmd.kd;
            double tau_ff = motor_cmd.tau;

            double pos_error = target_pos - current_pos;
            double vel_error = target_vel - current_vel;
            double torque_cmd = (kp * pos_error) + (kd * vel_error) + tau_ff;

            // 存储计算出的扭矩值
            if (i < jointForces_.size()) {
                jointForces_[i][0] = torque_cmd;
            } else {
                // 确保向量大小足够
                jointForces_.resize(i + 1, std::vector<double>(1, 0.0));
                jointForces_[i][0] = torque_cmd;
            }

            // Apply the calculated torque using OneTimeChange for efficiency
            _ecm.SetComponentData<ignition::gazebo::components::JointForceCmd>(
                jointEntity, {torque_cmd});
            ignition::common::Console::err << "Applying torque " << torque_cmd << " to " << jointNames_[i] << std::endl; // Debug
        }

        // Optional: Reset command after applying if you want each command to be used only once.
        // lastRosCmd_.reset();
    }

    // --- ReadAndPublishState (takes const ECM) ---
    void HectorGazeboFortressPlugin::ReadAndPublishState(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm)
    {
        // --- 1. Populate RobotState Message ---
        // Use _ecm.Component() which is const-correct for reading
        if (this->baseLinkEntity_ != ignition::gazebo::kNullEntity) {
            auto poseComp = _ecm.Component<ignition::gazebo::components::WorldPose>(this->baseLinkEntity_);
            auto linVelComp = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->baseLinkEntity_);
            auto angVelComp = _ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(this->baseLinkEntity_);

            // --- Body Pose ---
            if (poseComp) {
                const ignition::math::Pose3d& pose = poseComp->Data();
                // Check size before access
                if (robotStateMsg_.body_position.size() == 3) {
                    robotStateMsg_.body_position[0] = static_cast<float>(pose.Pos().X());
                    robotStateMsg_.body_position[1] = static_cast<float>(pose.Pos().Y());
                    robotStateMsg_.body_position[2] = static_cast<float>(pose.Pos().Z());
                }
                 // Use base link orientation only if IMU data hasn't been received
                if (!imuReceived_ && robotStateMsg_.imu.size() > 0 && robotStateMsg_.imu[0].quaternion.size() == 4) {
                    robotStateMsg_.imu[0].quaternion[0] = static_cast<float>(pose.Rot().X());
                    robotStateMsg_.imu[0].quaternion[1] = static_cast<float>(pose.Rot().Y());
                    robotStateMsg_.imu[0].quaternion[2] = static_cast<float>(pose.Rot().Z());
                    robotStateMsg_.imu[0].quaternion[3] = static_cast<float>(pose.Rot().W());
                }
            } else {
                 // If component missing, fill with zeros
                std::fill(robotStateMsg_.body_position.begin(), robotStateMsg_.body_position.end(), 0.0f);
                 if (!imuReceived_ && robotStateMsg_.imu.size() > 0) {
                     std::fill(robotStateMsg_.imu[0].quaternion.begin(), robotStateMsg_.imu[0].quaternion.end(), 0.0f);
                 }
                 // ignition::common::Console::warn << "WorldPose component missing for base link in ReadAndPublishState." << std::endl;
            }

            // --- Body Velocity ---
            if (linVelComp) {
                const ignition::math::Vector3d& lin_vel = linVelComp->Data();
                if (robotStateMsg_.body_velocity.size() == 3) {
                    robotStateMsg_.body_velocity[0] = static_cast<float>(lin_vel.X());
                    robotStateMsg_.body_velocity[1] = static_cast<float>(lin_vel.Y());
                    robotStateMsg_.body_velocity[2] = static_cast<float>(lin_vel.Z());
                }
            } else {
                std::fill(robotStateMsg_.body_velocity.begin(), robotStateMsg_.body_velocity.end(), 0.0f);
                // ignition::common::Console::warn << "WorldLinearVelocity component missing for base link in ReadAndPublishState." << std::endl;
            }

            // --- Body Angular Velocity (from world if IMU not available) ---
             if (angVelComp && !imuReceived_ && robotStateMsg_.imu.size() > 0 && robotStateMsg_.imu[0].gyroscope.size() == 3) {
                 const ignition::math::Vector3d& ang_vel = angVelComp->Data();
                 robotStateMsg_.imu[0].gyroscope[0] = static_cast<float>(ang_vel.X());
                 robotStateMsg_.imu[0].gyroscope[1] = static_cast<float>(ang_vel.Y());
                 robotStateMsg_.imu[0].gyroscope[2] = static_cast<float>(ang_vel.Z());
             }
             else if (!imuReceived_ && robotStateMsg_.imu.size() > 0) {
                 std::fill(robotStateMsg_.imu[0].gyroscope.begin(), robotStateMsg_.imu[0].gyroscope.end(), 0.0f);
                 // ignition::common::Console::warn << "WorldAngularVelocity component missing/IMU not received in ReadAndPublishState." << std::endl;
             }
        } else {
            // Base link entity is null, zero out relevant fields
            std::fill(robotStateMsg_.body_position.begin(), robotStateMsg_.body_position.end(), 0.0f);
            std::fill(robotStateMsg_.body_velocity.begin(), robotStateMsg_.body_velocity.end(), 0.0f);
            if (robotStateMsg_.imu.size() > 0) {
                std::fill(robotStateMsg_.imu[0].quaternion.begin(), robotStateMsg_.imu[0].quaternion.end(), 0.0f);
                std::fill(robotStateMsg_.imu[0].gyroscope.begin(), robotStateMsg_.imu[0].gyroscope.end(), 0.0f);
                std::fill(robotStateMsg_.imu[0].accelerometer.begin(), robotStateMsg_.imu[0].accelerometer.end(), 0.0f);
            }
        }

        // --- IMU Data (from Ignition Transport callback) ---
        { // IMU Mutex Scope
            std::lock_guard<std::mutex> lock(imuMutex_);
            if (imuReceived_ && robotStateMsg_.imu.size() > 0) {
                // Orientation (Quaternion)
                if (lastIgnImuMsg_.has_orientation()) {
                     robotStateMsg_.imu[0].quaternion[0] = static_cast<float>(lastIgnImuMsg_.orientation().x());
                     robotStateMsg_.imu[0].quaternion[1] = static_cast<float>(lastIgnImuMsg_.orientation().y());
                     robotStateMsg_.imu[0].quaternion[2] = static_cast<float>(lastIgnImuMsg_.orientation().z());
                     robotStateMsg_.imu[0].quaternion[3] = static_cast<float>(lastIgnImuMsg_.orientation().w());
                }
                // Gyroscope
                if (lastIgnImuMsg_.has_angular_velocity()) {
                    robotStateMsg_.imu[0].gyroscope[0] = static_cast<float>(lastIgnImuMsg_.angular_velocity().x());
                    robotStateMsg_.imu[0].gyroscope[1] = static_cast<float>(lastIgnImuMsg_.angular_velocity().y());
                    robotStateMsg_.imu[0].gyroscope[2] = static_cast<float>(lastIgnImuMsg_.angular_velocity().z());
                }
                // Accelerometer
                if (lastIgnImuMsg_.has_linear_acceleration()) {
                    robotStateMsg_.imu[0].accelerometer[0] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().x());
                    robotStateMsg_.imu[0].accelerometer[1] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().y());
                    robotStateMsg_.imu[0].accelerometer[2] = static_cast<float>(lastIgnImuMsg_.linear_acceleration().z());
                }
                 // Reset flag? Decide if IMU data should persist if messages stop
                 // imuReceived_ = false; // Uncomment if you want to fall back to base link data if IMU stops
            } else if (robotStateMsg_.imu.size() > 0) {
                // Ensure accel is zeroed if no IMU data ever received
                 std::fill(robotStateMsg_.imu[0].accelerometer.begin(), robotStateMsg_.imu[0].accelerometer.end(), 0.0f);
            }
        } // End IMU Mutex Scope
        // --- Joint States ---
        if (robotStateMsg_.motor_state.size() == jointEntities_.size()) {
            for (size_t i = 0; i < jointEntities_.size(); ++i) {
                ignition::gazebo::Entity jointEntity = jointEntities_[i];
                auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
                auto velComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);

                // Position
                if (posComp && !posComp->Data().empty()) {
                    robotStateMsg_.motor_state[i].q = static_cast<float>(posComp->Data()[0]);
                } else {
                    robotStateMsg_.motor_state[i].q = 0.0f; // Default value
                }
                // Velocity
                if (velComp && !velComp->Data().empty()) {
                    robotStateMsg_.motor_state[i].dq = static_cast<float>(velComp->Data()[0]);
                } else {
                    robotStateMsg_.motor_state[i].dq = 0.0f; // Default value
                }

                // 使用存储的力值而不是尝试读取可能已经被清零的JointForce组件
                if (i < jointForces_.size() && !jointForces_[i].empty()) {
                    robotStateMsg_.motor_state[i].tauest = static_cast<float>(jointForces_[i][0]);
                    ignition::common::Console::err << "name:" << jointNames_[i] << " force:" << static_cast<float>(jointForces_[i][0]) << std::endl;

                } else {
                    robotStateMsg_.motor_state[i].tauest = 0.0f; // Default value
                    ignition::common::Console::err << "No stored force value for joint " << jointNames_[i] << " when reading tauEst." << std::endl;
                }
            }
        }

        // --- 2. Populate ContactState Message ---
        { // Contact Mutex Scope
            std::lock_guard<std::mutex> lock(contactMutex_);
            if (contactStateMsg_.contact_state.size() == 2) {
                contactStateMsg_.contact_state[0] = leftContact_ ? 1.0f : 0.0f;
                contactStateMsg_.contact_state[1] = rightContact_ ? 1.0f : 0.0f;
            }
            // Reset flags *after* reading them for this timestep's publication
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
        ignition::common::Console::err << "Received new ROS command." << std::endl; // Use log or debug level
    }

    void HectorGazeboFortressPlugin::IgnImuCallback(const ignition::msgs::IMU &_msg)
    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        lastIgnImuMsg_ = _msg;
        imuReceived_ = true;
    }

    void HectorGazeboFortressPlugin::IgnLeftContactCallback(const ignition::msgs::Contacts &_msg)
    {
        // Check for contact with the specified ground collision object
        bool contact_with_ground = false;
        for (int i = 0; i < _msg.contact_size(); ++i) {
            const auto& contact = _msg.contact(i);
            // Check both collision objects involved in the contact
            if (contact.collision1().name().find(this->groundCollisionName_) != std::string::npos ||
                contact.collision2().name().find(this->groundCollisionName_) != std::string::npos)
            {
                 // Ensure the *other* collision isn't also the ground (unlikely but possible)
                // This assumes the contact sensor is attached to the foot/toe link
                if (contact.collision1().name().find(this->groundCollisionName_) == std::string::npos ||
                    contact.collision2().name().find(this->groundCollisionName_) == std::string::npos)
                {
                    contact_with_ground = true;
                    break; // Found one valid ground contact
                }
            }
        }

        // Set the flag if contact occurred (will be read in PostUpdate)
        if (contact_with_ground) {
            std::lock_guard<std::mutex> lock(contactMutex_);
            leftContact_ = true;
            // ignition::common::Console::log << "Left Contact detected." << std::endl;
        }
        // Note: No 'else' here, the flag is reset in PostUpdate after publishing
    }

    void HectorGazeboFortressPlugin::IgnRightContactCallback(const ignition::msgs::Contacts &_msg)
    {
        // Check for contact with the specified ground collision object
        bool contact_with_ground = false;
        for (int i = 0; i < _msg.contact_size(); ++i) {
            const auto& contact = _msg.contact(i);
            if (contact.collision1().name().find(this->groundCollisionName_) != std::string::npos ||
               contact.collision2().name().find(this->groundCollisionName_) != std::string::npos)
            {
                if (contact.collision1().name().find(this->groundCollisionName_) == std::string::npos ||
                    contact.collision2().name().find(this->groundCollisionName_) == std::string::npos)
                {
                    contact_with_ground = true;
                    break;
                }
            }
        }

        if (contact_with_ground) {
            std::lock_guard<std::mutex> lock(contactMutex_);
            rightContact_ = true;
             // ignition::common::Console::log << "Right Contact detected." << std::endl;
        }
    }
} // namespace hector_gazebo_plugins

// --- Plugin Registration ---
IGNITION_ADD_PLUGIN(hector_gazebo_plugins::HectorGazeboFortressPlugin,
              ignition::gazebo::System,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemConfigure,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemPreUpdate,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemUpdate,
              hector_gazebo_plugins::HectorGazeboFortressPlugin::ISystemPostUpdate) // <-- Added PostUpdate Interface registration