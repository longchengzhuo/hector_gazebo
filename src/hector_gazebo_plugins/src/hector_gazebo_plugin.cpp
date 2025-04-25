#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh> // Gazebo transport (optional, but good practice)
#include <gazebo_ros/node.hpp> // For ROS 2 node integration
#include <rclcpp/rclcpp.hpp>
#include <laser_interfaces/msg/robot_command.hpp> // <--- Added for RobotCommand
#include <laser_interfaces/msg/motor_command.hpp> // Still needed for the type within RobotCommand

#include <string>
#include <vector>
#include <mutex>
#include <memory> // For std::shared_ptr


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
        // --- Gazebo Initialization ---
        model_ = _model;
        world_ = _model->GetWorld();

        if (!world_) {
            gzerr << "Parent world is null! Plugin could not be loaded." << std::endl;
            return;
        }
        gzmsg << "Loading HectorGazeboPlugin for model: " << _model->GetName() << std::endl;

        // --- ROS 2 Initialization ---
        ros_node_ = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "Initializing Hector Gazebo Plugin (ROS Node)");

        // --- Get Joint Names from SDF ---
        joint_names_.clear();
        if (!_sdf->HasElement("joint_name"))
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "No <joint_name> elements found in SDF. Plugin cannot control joints.");
            return;
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
             return;
        }
        num_joints_ = joint_names_.size();
        RCLCPP_INFO(ros_node_->get_logger(), "Will control %zu joints.", num_joints_);

        // --- Get Joint Pointers from Gazebo Model ---
        joints_.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) {
            joints_[i] = model_->GetJoint(joint_names_[i]);
            if (!joints_[i]) {
                RCLCPP_ERROR(ros_node_->get_logger(), "Could not find joint '%s' in the model.", joint_names_[i].c_str());
                return; // Or handle appropriately
            }
        }

        // --- ROS 2 Subscription ---
        // Topic name should match the publisher in the controller node sending RobotCommand
        std::string topic_name = "/Hector_Command"; // Default topic name
        if (_sdf->HasElement("topic_name")) {
            topic_name = _sdf->Get<std::string>("topic_name");
            RCLCPP_INFO(ros_node_->get_logger(), "Using custom topic name from SDF: %s", topic_name.c_str());
        }

        // Subscribe to RobotCommand instead of MotorCommand
        cmd_sub_ = ros_node_->create_subscription<laser_interfaces::msg::RobotCommand>( // <--- Changed message type
            topic_name,
            10, // QoS depth
            std::bind(&HectorGazeboPlugin::CommandCallback, this, std::placeholders::_1));

        RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to %s for RobotCommand messages", topic_name.c_str());

        // --- Gazebo Update Connection ---
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&HectorGazeboPlugin::OnUpdate, this));

        RCLCPP_INFO(ros_node_->get_logger(), "Hector Gazebo Plugin loaded successfully.");
    }

private:
    // Called every simulation step by Gazebo
    void OnUpdate()
    {
        // Lock mutex to safely access last_cmd_
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        if (!last_cmd_) {
            // No command received yet, apply zero force
             for (size_t i = 0; i < num_joints_; ++i) {
                 if (joints_[i]) {
                     joints_[i]->SetForce(0, 0.0); // Apply zero force if no command
                 }
             }
            return;
        }

        // Ensure the command has the correct number of motor commands
        // Use 'motor_command' field from RobotCommand message
        if (last_cmd_->motor_command.size() != num_joints_) {
            RCLCPP_WARN_ONCE(ros_node_->get_logger(),
                "Received RobotCommand message with %zu motor entries, but plugin controls %zu joints. Skipping command.",
                last_cmd_->motor_command.size(), num_joints_);
             // Apply zero force to prevent unexpected behavior
              for (size_t i = 0; i < num_joints_; ++i) {
                 if (joints_[i]) {
                     joints_[i]->SetForce(0, 0.0);
                 }
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
            // Use 'motor_command' field from RobotCommand message
            const auto& motor_cmd = last_cmd_->motor_command[i]; // <--- Changed 'motors' to 'motor_command'
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
        // Optional: If you want to apply the command only once per message,
        // you could uncomment the next line:
        // last_cmd_.reset();
    }

    // Called when a new command message is received
    // Callback parameter type changed to RobotCommand
    void CommandCallback(const laser_interfaces::msg::RobotCommand::SharedPtr msg) // <--- Changed message type
    {
        // Lock mutex to safely update last_cmd_
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = msg;
        // Optional: Add a timestamp check if needed
        // last_cmd_time_ = world_->SimTime();
        // RCLCPP_DEBUG(ros_node_->get_logger(), "Received new robot command.");
    }

    // --- Member Variables ---
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    size_t num_joints_ = 0;

    // ROS 2 related
    gazebo_ros::Node::SharedPtr ros_node_;
    // Subscription type changed to RobotCommand
    rclcpp::Subscription<laser_interfaces::msg::RobotCommand>::SharedPtr cmd_sub_; // <--- Changed message type
    // Last command type changed to RobotCommand
    laser_interfaces::msg::RobotCommand::SharedPtr last_cmd_; // <--- Changed message type
    std::mutex cmd_mutex_; // Mutex to protect access to last_cmd_

    // Gazebo update event connection
    event::ConnectionPtr update_connection_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HectorGazeboPlugin)
} // namespace gazebo