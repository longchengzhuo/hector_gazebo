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
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
public:
    HectorGazeboFortressPlugin();
    virtual ~HectorGazeboFortressPlugin() override;

    // --- System Interface Implementations ---
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // <-- Added PostUpdate Declaration -->
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    // --- Helper Functions ---
    bool ParseSDF(const std::shared_ptr<const sdf::Element> &_sdf);
    void InitROS();
    void InitIgnitionTransport();
    bool FindEntities(ignition::gazebo::EntityComponentManager &_ecm);
    void ApplyControl(ignition::gazebo::EntityComponentManager &_ecm);
    // Changed _ecm to const as PostUpdate provides const access
    void ReadAndPublishState(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm);

    // --- Callbacks ---
    void RosCommandCallback(const laser_interfaces::msg::RobotCommand::SharedPtr _msg);
    void IgnImuCallback(const ignition::msgs::IMU &_msg);
    void IgnLeftContactCallback(const ignition::msgs::Contacts &_msg);
    void IgnRightContactCallback(const ignition::msgs::Contacts &_msg);


    // --- Member Variables ---
    // Model and Entities
    ignition::gazebo::Entity modelEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity baseLinkEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity lFootLInkEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity rFootLInkEntity_{ignition::gazebo::kNullEntity};
    std::vector<ignition::gazebo::Entity> jointEntities_;
    std::vector<std::string> jointNames_;
    std::string baseLinkName_;
    std::string lFootLInkName_;
    std::string rFootLInkName_;

    size_t numJoints_{0};
    std::vector<std::vector<double>> jointForces_;

    // SDF Parameters
    std::string rosCmdTopic_;
    std::string rosRobotStateTopic_;
    std::string rosContactStateTopic_;
    std::string ignImuTopic_;
    std::string ignLeftContactTopic_;
    std::string ignRightContactTopic_;
    std::string groundCollisionName_;

    // ROS 2
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spinThread_;
    std::atomic<bool> shouldSpin_{false};

    rclcpp::Node::SharedPtr rosNode_;
    rclcpp::Subscription<laser_interfaces::msg::RobotCommand>::SharedPtr rosCmdSub_;
    rclcpp::Publisher<laser_interfaces::msg::RobotState>::SharedPtr rosRobotStatePub_;
    rclcpp::Publisher<laser_interfaces::msg::ContactState>::SharedPtr rosContactStatePub_;
    std::optional<laser_interfaces::msg::RobotCommand> lastRosCmd_;
    std::mutex cmdMutex_;

    ignition::transport::Node ignNode_;

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

    // 基于仿真时间的性能计时器
    std::chrono::steady_clock::duration last_sim_time_;
    double average_sim_publish_period_ = 0.0;
    size_t publish_count_ = 0;
    bool first_publish_ = true;
};

}

#endif