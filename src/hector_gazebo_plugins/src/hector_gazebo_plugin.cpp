#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh> // Added for Link access
#include <gazebo/physics/World.hh> // Added for World access
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/sensors/SensorManager.hh> // Added for Sensor access
#include <gazebo/sensors/ImuSensor.hh>      // Added for IMU Sensor
#include <gazebo/sensors/ContactSensor.hh> // Added for Contact Sensor
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <laser_interfaces/msg/robot_command.hpp>
#include <laser_interfaces/msg/motor_command.hpp>
#include <laser_interfaces/msg/robot_state.hpp>   // <--- Added for RobotState
#include <laser_interfaces/msg/contact_state.hpp> // <--- Added for ContactState
#include <ignition/math/Vector3.hh>             // Added for Gazebo math types
#include <ignition/math/Pose3.hh>               // Added for Gazebo math types
#include <ignition/math/Quaternion.hh>          // Added for Gazebo math types


#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace gazebo
{

class HectorGazeboPlugin : public ModelPlugin
{
public:
    HectorGazeboPlugin() = default;
    virtual ~HectorGazeboPlugin() {
        update_connection_.reset(); // Disconnect update event listener
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // --- ROS 2 Initialization ---
        model_ = _model;
        ros_node_ = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "Initializing Hector Gazebo Plugin (ROS Node) for model: %s", model_->GetName().c_str());

        // ------------------------------------------ Get Base Link ----------------------------------------------
        base_link_name_ = "base_link";
        base_link_ = model_->GetLink(base_link_name_);
        if (!base_link_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Could not find base link '%s' in the model.", base_link_name_.c_str());
            return;
        }
        RCLCPP_INFO(ros_node_->get_logger(), "Using base link: %s", base_link_name_.c_str());


        // ------------------------------------- Get Joint Names and Pointers -------------------------------------
        if (!LoadJoints(_sdf)) return; // Refactored joint loading

        // -------------------------------- Get Sensor Names from SDF (IMU and Contact) ---------------------------

        // --- ROS 2 Subscription ---
        SetupSubscription(); // Refactored subscription setup

        // --- ROS 2 Publishers ---
        SetupPublisher();

        // --- Gazebo Update Connection ---
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&HectorGazeboPlugin::OnUpdate, this));

        RCLCPP_INFO(ros_node_->get_logger(), "Hector Gazebo Plugin loaded successfully.");
    }

private:
    // --- Helper Functions ---

    // Load joint names from SDF and get pointers
    bool LoadJoints(const sdf::ElementPtr& _sdf) {
        joint_names_.clear();
        if (!_sdf->HasElement("joint_name"))
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "No <joint_name> elements found in SDF. Plugin cannot control joints.");
            return false;
        }
        sdf::ElementPtr joint_elem = _sdf->GetElement("joint_name");
        while (joint_elem) {
            std::string joint_name = joint_elem->Get<std::string>();
            joint_names_.push_back(joint_name);
            RCLCPP_INFO(ros_node_->get_logger(), "Found joint to control: %s", joint_name.c_str());
            joint_elem = joint_elem->GetNextElement("joint_name");
        }

        if (joint_names_.empty()) {
             RCLCPP_ERROR(ros_node_->get_logger(), "Joint names list is empty after parsing SDF.");
             return false;
        }
        num_joints_ = joint_names_.size();
        RCLCPP_INFO(ros_node_->get_logger(), "Will control %zu joints.", num_joints_);

        // Get Joint Pointers from Gazebo Model
        joints_.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) {
            joints_[i] = model_->GetJoint(joint_names_[i]);
            if (!joints_[i]) {
                RCLCPP_ERROR(ros_node_->get_logger(), "Could not find joint '%s' in the model.", joint_names_[i].c_str());
                return false;
            }
        }

        return true;
    }

    bool LoadSensors(const sdf::ElementPtr& _sdf) {
        // IMU Sensor (Required)
        if (!_sdf->HasElement("imu_sensor_name")) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Missing required <imu_sensor_name> element in SDF.");
            return false;
        }
        imu_sensor_name_ = _sdf->Get<std::string>("imu_sensor_name");

        // Contact Sensors (Required)
        if (!_sdf->HasElement("left_toe_contact_sensor_name")) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Missing required <left_toe_contact_sensor_name> element in SDF.");
            return false;
        }
        left_toe_contact_sensor_name_ = _sdf->Get<std::string>("left_toe_contact_sensor_name");

        if (!_sdf->HasElement("right_toe_contact_sensor_name")) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Missing required <right_toe_contact_sensor_name> element in SDF.");
            return false;
        }
        right_toe_contact_sensor_name_ = _sdf->Get<std::string>("right_toe_contact_sensor_name");

        return true;
    }

    // Set up ROS 2 subscription for commands
    void SetupSubscription() {
        std::string topic_name = "/Hector_Command"; // Default topic name

        // Subscribe to RobotCommand instead of MotorCommand
        cmd_sub_ = ros_node_->create_subscription<laser_interfaces::msg::RobotCommand>( // <--- Changed message type
            topic_name,
            10, // QoS depth
            std::bind(&HectorGazeboPlugin::CommandCallback, this, std::placeholders::_1));

        RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to %s for RobotCommand messages", topic_name.c_str());
    }

    void SetupPublisher() {
        std::string robot_state_topic = "/Hector_State"; // Default
        robot_state_pub_ = ros_node_->create_publisher<laser_interfaces::msg::RobotState>(
            robot_state_topic, 10);
        RCLCPP_INFO(ros_node_->get_logger(), "Publishing RobotState on topic: %s", robot_state_topic.c_str());

        // Contact State Publisher
        std::string contact_state_topic = "/true_toe_floor_contact"; // Default
        contact_state_pub_ = ros_node_->create_publisher<laser_interfaces::msg::ContactState>(
            contact_state_topic, 10);
        RCLCPP_INFO(ros_node_->get_logger(), "Publishing ContactState on topic: %s", contact_state_topic.c_str());
    }

    // Get Sensor Pointers (call once after sensors are initialized)
    void FindSensorPointers() {
         sensors::SensorManager* sensor_mgr = sensors::SensorManager::Instance();

        // Find IMU Sensor
        imu_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor_mgr->GetSensor(imu_sensor_name_));
        if (!imu_sensor_) {
            RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000, // Log every 5s
                "Could not find IMU sensor named '%s'. State publishing will be incomplete.", imu_sensor_name_.c_str());
        } else {
            RCLCPP_INFO(ros_node_->get_logger(), "Found IMU sensor: %s", imu_sensor_name_.c_str());
        }
        

        // Find Left Toe Contact Sensor
        left_toe_contact_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor_mgr->GetSensor(left_toe_contact_sensor_name_));
        if (!left_toe_contact_sensor_) {
         RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
            "Could not find Left Toe Contact sensor named '%s'. Contact state will be incomplete.", left_toe_contact_sensor_name_.c_str());
        } else {
         RCLCPP_INFO(ros_node_->get_logger(), "Found Left Toe Contact sensor: %s", left_toe_contact_sensor_name_.c_str());
         left_toe_contact_sensor_->SetActive(true); // Ensure active
        }
         

         // Find Right Toe Contact Sensor
        right_toe_contact_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor_mgr->GetSensor(right_toe_contact_sensor_name_));
        if (!right_toe_contact_sensor_) {
        RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
            "Could not find Right Toe Contact sensor named '%s'. Contact state will be incomplete.", right_toe_contact_sensor_name_.c_str());
        } else {
         RCLCPP_INFO(ros_node_->get_logger(), "Found Right Toe Contact sensor: %s", right_toe_contact_sensor_name_.c_str());
         right_toe_contact_sensor_->SetActive(true); // Ensure active
        }
         
    }


    // Called every simulation step by Gazebo
    void OnUpdate()
    {
        // Ensure sensor pointers are valid (sensors might initialize after the plugin loads)
        FindSensorPointers();

        // 1. Apply Control Command (from CommandCallback)
        ApplyControl();

        // 2. Read State from Gazebo
        ReadAndPopulateState();

        // 3. Publish State
        PublishState();
    }

    // Apply forces based on the last received command
    void ApplyControl() {
         // Lock mutex to safely access last_cmd_
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        if (!last_cmd_) {
            // No command received yet, apply zero force (optional, prevents drift)
            // for (size_t i = 0; i < num_joints_; ++i) {
            //     if (joints_[i]) joints_[i]->SetForce(0, 0.0);
            // }
            return; // Do nothing if no command received
        }

        // Ensure the command has the correct number of motor commands
        if (last_cmd_->motor_command.size() != num_joints_) {
            RCLCPP_WARN_ONCE(ros_node_->get_logger(),
                "Received RobotCommand with %zu motor entries, but plugin controls %zu joints. Skipping command application.",
                last_cmd_->motor_command.size(), num_joints_);
             // Apply zero force to prevent unexpected behavior if size mismatch
              for (size_t i = 0; i < num_joints_; ++i) {
                 if (joints_[i]) joints_[i]->SetForce(0, 0.0);
              }
            return;
        }

        // Apply forces based on the last received command
        for (size_t i = 0; i < num_joints_; ++i)
        {
            if (!joints_[i]) continue; // Skip if joint pointer is invalid

            // Get current state from Gazebo
            double current_pos = joints_[i]->Position(0); // Axis 0 for revolute/prismatic
            double current_vel = joints_[i]->GetVelocity(0);

            // Get targets and gains from the command message's motor_command array
            const auto& motor_cmd = last_cmd_->motor_command[i];
            double target_pos = motor_cmd.q;
            double target_vel = motor_cmd.dq;
            double kp = motor_cmd.kp;
            double kd = motor_cmd.kd;
            double tau_ff = motor_cmd.tau; // Feed-forward torque

            // Calculate PD control torque + feed-forward
            double pos_error = target_pos - current_pos;
            double vel_error = target_vel - current_vel;
            double torque_cmd = (kp * pos_error) + (kd * vel_error) + tau_ff;

            // Apply the calculated force/torque to the joint
            joints_[i]->SetForce(0, torque_cmd); // Axis 0
        }
    }

    // Read sensor data and populate the state messages
    void ReadAndPopulateState() {
        // --- Populate RobotState ---
        // Body Pose and Velocity (from base link)
        if (base_link_) {
             ignition::math::Pose3d pose = base_link_->WorldPose();
             ignition::math::Vector3d lin_vel = base_link_->WorldLinearVel();
            //  ignition::math::Vector3d ang_vel = base_link_->WorldAngularVel(); // Can get from link too

             robot_state_msg_.body_position[0] = pose.Pos().X();
             robot_state_msg_.body_position[1] = pose.Pos().Y();
             robot_state_msg_.body_position[2] = pose.Pos().Z();

             robot_state_msg_.body_velocity[0] = lin_vel.X();
             robot_state_msg_.body_velocity[1] = lin_vel.Y();
             robot_state_msg_.body_velocity[2] = lin_vel.Z();

             // Use IMU for orientation, accel, gyro if available, otherwise base link orientation
             if (!imu_sensor_) {
                 robot_state_msg_.imu[0].quaternion[0] = pose.Rot().X();
                 robot_state_msg_.imu[0].quaternion[1] = pose.Rot().Y();
                 robot_state_msg_.imu[0].quaternion[2] = pose.Rot().Z();
                 robot_state_msg_.imu[0].quaternion[3] = pose.Rot().W();
             }
        } else {
            // Log error or set default values if base_link_ is null
             RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000, "Base link pointer is null. Cannot get body pose/velocity.");
        }


        // IMU Data (from IMU sensor) - Overwrites orientation if sensor exists
        if (imu_sensor_) {
            ignition::math::Quaterniond orientation = imu_sensor_->Orientation(); // World frame orientation
            ignition::math::Vector3d angular_vel = imu_sensor_->AngularVelocity(); // Sensor frame
            ignition::math::Vector3d linear_accel = imu_sensor_->LinearAcceleration(); // Sensor frame

            // Assuming the RobotState expects world frame orientation and body frame velocities/accel
            // (Adjust if message definition implies otherwise)
            robot_state_msg_.imu[0].quaternion[0] = orientation.X();
            robot_state_msg_.imu[0].quaternion[1] = orientation.Y();
            robot_state_msg_.imu[0].quaternion[2] = orientation.Z();
            robot_state_msg_.imu[0].quaternion[3] = orientation.W();

            robot_state_msg_.imu[0].gyroscope[0] = angular_vel.X();
            robot_state_msg_.imu[0].gyroscope[1] = angular_vel.Y();
            robot_state_msg_.imu[0].gyroscope[2] = angular_vel.Z();

            robot_state_msg_.imu[0].accelerometer[0] = linear_accel.X();
            robot_state_msg_.imu[0].accelerometer[1] = linear_accel.Y();
            robot_state_msg_.imu[0].accelerometer[2] = linear_accel.Z();
        } else {
             // Log warning or set default values if imu_sensor_ is null
             // Orientation might be partially filled by base_link_ above
            std::fill(robot_state_msg_.imu[0].gyroscope.begin(), robot_state_msg_.imu[0].gyroscope.end(), 0.0);
            std::fill(robot_state_msg_.imu[0].accelerometer.begin(), robot_state_msg_.imu[0].accelerometer.end(), 0.0);
        }

        // Joint States (Position, Velocity, Effort/Torque)
        for (size_t i = 0; i < num_joints_; ++i) {
            if (joints_[i]) {
                 robot_state_msg_.motor_state[i].q = joints_[i]->Position(0);
                 robot_state_msg_.motor_state[i].dq = joints_[i]->GetVelocity(0);
                 // GetForce(0) returns the last *applied* force/torque.
                 // This is the closest standard Gazebo value to MuJoCo's `tauest` sensor
                 // without adding specific joint torque sensors in Gazebo.
                 robot_state_msg_.motor_state[i].tauest = joints_[i]->GetForce(0);
            } else {
                // Set default values if joint pointer is invalid (should not happen after Load)
                robot_state_msg_.motor_state[i].q = 0.0;
                robot_state_msg_.motor_state[i].dq = 0.0;
                robot_state_msg_.motor_state[i].tauest = 0.0;
            }
        }

        // --- Populate ContactState ---
        // Reset states at the beginning of each update
        contact_state_msg_.contact_state[0] = 0.0f; // Left toe
        contact_state_msg_.contact_state[1] = 0.0f; // Right toe


        const std::string ground_collision_name = "ground_plane::link::collision";

        // Check Left Toe Contact
        if (left_toe_contact_sensor_) {
            msgs::Contacts contacts = left_toe_contact_sensor_->Contacts();
            for (int i = 0; i < contacts.contact_size(); ++i) {
                const msgs::Contact& contact = contacts.contact(i);
                if (contact.collision1() == ground_collision_name ||
                    contact.collision2() == ground_collision_name)
                {
                    contact_state_msg_.contact_state[0] = 1.0f;
                    break;
                }
            }
        }

        // Check Right Toe Contact
        if (right_toe_contact_sensor_) {
            msgs::Contacts contacts = right_toe_contact_sensor_->Contacts();
            for (int i = 0; i < contacts.contact_size(); ++i) {
                const msgs::Contact& contact = contacts.contact(i);
                if (contact.collision1() == ground_collision_name ||
                    contact.collision2() == ground_collision_name)
                {
                    contact_state_msg_.contact_state[1] = 1.0f;
                    break;
                }
            }
        }
    }

    // Publish the populated state messages
    void PublishState() {
        robot_state_pub_->publish(robot_state_msg_);
        contact_state_pub_->publish(contact_state_msg_);
    }


    // Called when a new command message is received
    void CommandCallback(const laser_interfaces::msg::RobotCommand::SharedPtr msg)
    {
        // Lock mutex to safely update last_cmd_
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = msg;
        // RCLCPP_DEBUG(ros_node_->get_logger(), "Received new robot command."); // Optional debug
    }

    // --- Member Variables ---
    // Gazebo
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    std::string base_link_name_;
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    size_t num_joints_ = 0;
    event::ConnectionPtr update_connection_;

    // Gazebo Sensors
    std::string imu_sensor_name_;
    std::string left_toe_contact_sensor_name_;
    std::string right_toe_contact_sensor_name_;
    sensors::ImuSensorPtr imu_sensor_;
    sensors::ContactSensorPtr left_toe_contact_sensor_;
    sensors::ContactSensorPtr right_toe_contact_sensor_;


    // ROS 2 related
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<laser_interfaces::msg::RobotCommand>::SharedPtr cmd_sub_;
    laser_interfaces::msg::RobotCommand::SharedPtr last_cmd_;
    std::mutex cmd_mutex_; // Protects last_cmd_

    rclcpp::Publisher<laser_interfaces::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<laser_interfaces::msg::ContactState>::SharedPtr contact_state_pub_;

    // Pre-allocated messages to avoid allocation in OnUpdate
    laser_interfaces::msg::RobotState robot_state_msg_;
    laser_interfaces::msg::ContactState contact_state_msg_;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HectorGazeboPlugin)
} // namespace gazebo