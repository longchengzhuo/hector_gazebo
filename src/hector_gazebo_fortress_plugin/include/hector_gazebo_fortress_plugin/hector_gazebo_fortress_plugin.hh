#ifndef HECTOR_GAZEBO_FORTRESS_PLUGIN_HH_
#define HECTOR_GAZEBO_FORTRESS_PLUGIN_HH_


#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh> // Included for Entity usage (like kNullEntity)
#include <ignition/gazebo/Types.hh>         // Included for UpdateInfo
#include <ignition/gazebo/components/JointVelocityCmd.hh> // For applying joint commands
#include <ignition/gazebo/components/ContactSensor.hh> // If accessing contact data via components

// --- Ignition Transport & Msgs Includes ---
#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/imu.pb.h>

// --- ROS 2 Includes ---
#include <rclcpp/rclcpp.hpp>
#include <laser_interfaces/msg/robot_command.hpp>
#include <laser_interfaces/msg/robot_state.hpp>
#include <laser_interfaces/msg/contact_state.hpp>

// --- Standard Library Includes ---
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <optional>

// --- Namespace Definition ---
namespace hector_gazebo_plugins
{

class HectorGazeboFortressPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
    HectorGazeboFortressPlugin();
    virtual ~HectorGazeboFortressPlugin() override;

    // --- System Interface Implementations ---
    // ISystemConfigure uses the full ignition::gazebo::EntityComponentManager type
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm, // Needs full definition
                   ignition::gazebo::EventManager &_eventMgr) override;

    // ISystemPreUpdate uses the full ignition::gazebo::EntityComponentManager type
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override; // Needs full definition

private:
    // --- Helper Functions ---
    bool ParseSDF(const std::shared_ptr<const sdf::Element> &_sdf);
    void InitROS();
    void InitIgnitionTransport();
    // Uses ignition::gazebo::EntityComponentManager
    bool FindEntities(ignition::gazebo::EntityComponentManager &_ecm); // Needs full definition
    // Uses ignition::gazebo::EntityComponentManager
    void ApplyControl(ignition::gazebo::EntityComponentManager &_ecm); // Needs full definition
    // Uses ignition::gazebo::EntityComponentManager
    void ReadAndPublishState(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm); // Needs full definition

    // --- Callbacks ---
    void RosCommandCallback(const laser_interfaces::msg::RobotCommand::SharedPtr _msg);
    void IgnImuCallback(const ignition::msgs::IMU &_msg);
    void IgnLeftContactCallback(const ignition::msgs::Contacts &_msg);
    void IgnRightContactCallback(const ignition::msgs::Contacts &_msg);


    // --- Member Variables ---
    // Model and Entities
    ignition::gazebo::Entity modelEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity baseLinkEntity_{ignition::gazebo::kNullEntity};
    std::vector<ignition::gazebo::Entity> jointEntities_;
    std::vector<std::string> jointNames_;
    std::string baseLinkName_;
    size_t numJoints_{0};

    // SDF Parameters
    std::string rosCmdTopic_;
    std::string rosRobotStateTopic_;
    std::string rosContactStateTopic_;
    std::string ignImuTopic_;
    std::string ignLeftContactTopic_;
    std::string ignRightContactTopic_;
    std::string groundCollisionName_;

    // ROS 2
    rclcpp::Node::SharedPtr rosNode_;
    rclcpp::Subscription<laser_interfaces::msg::RobotCommand>::SharedPtr rosCmdSub_;
    rclcpp::Publisher<laser_interfaces::msg::RobotState>::SharedPtr rosRobotStatePub_;
    rclcpp::Publisher<laser_interfaces::msg::ContactState>::SharedPtr rosContactStatePub_;
    std::optional<laser_interfaces::msg::RobotCommand> lastRosCmd_;
    std::mutex cmdMutex_;

    // Ignition Transport
    ignition::transport::Node ignNode_;
    ignition::transport::Node::Publisher ignJointCmdPub_; // Example: If publishing joint commands via ign transport
    // Add subscriptions if needed for IMU/Contact if not handled by callbacks directly
    // ignition::transport::Node::Subscription<ignition::msgs::IMU>::SharedPtr ignImuSub_;
    // ignition::transport::Node::Subscription<ignition::msgs::Contacts>::SharedPtr ignLeftContactSub_;
    // ignition::transport::Node::Subscription<ignition::msgs::Contacts>::SharedPtr ignRightContactSub_;


    // State Storage
    std::mutex imuMutex_;
    ignition::msgs::IMU lastIgnImuMsg_;
    bool imuReceived_{false};

    std::mutex contactMutex_;
    bool leftContact_{false};
    bool rightContact_{false};

    // Pre-allocated messages
    laser_interfaces::msg::RobotState robotStateMsg_;
    laser_interfaces::msg::ContactState contactStateMsg_;

    // Initialization flags
    bool rosInitialized_{false};
    bool entitiesFound_{false};
    bool ignTransportInitialized_{false};
    bool sdfParsed_{false};
};

} // namespace hector_gazebo_plugins

#endif // HECTOR_GAZEBO_FORTRESS_PLUGIN_HH_