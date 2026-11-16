#include "padflies_cpp/commander/actor/ll_commander_minimal.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

LowLevelCommanderMinimal::LowLevelCommanderMinimal(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface, 
    const std::string & cf_prefix)
: m_cf_prefix(cf_prefix)
, m_logger(node_logging_interface->get_logger())
{
    if (m_callback_groups.find(cf_prefix) == m_callback_groups.end())
        m_callback_groups[cf_prefix] = node_base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_groups[cf_prefix];

    m_notify_setpoints_stop_client = rclcpp::create_client<crazyflie_interfaces::srv::NotifySetpointsStop>(
        node_base_interface, 
        node_graph_interface, 
        node_services_interface, 
        cf_prefix + "/notify_setpoints_stop",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);

    m_cmd_position_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::Position>(
        node_topics_interface, 
        m_cf_prefix + "/cmd_position", 
        10, 
        pub_options);

    m_cmd_velocity_world_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::VelocityWorld>(
        node_topics_interface, 
        m_cf_prefix + "/cmd_velocity_world", 
        10, 
        pub_options);
}

LowLevelCommanderMinimal::~LowLevelCommanderMinimal()
{
    m_notify_setpoints_stop_client.reset();
    m_cmd_position_pub.reset();
    m_cmd_velocity_world_pub.reset();
    // m_callback_group.reset(); // See note above about m_callback_group
    RCLCPP_DEBUG(m_logger, "LowLevelCommanderMinimal destructor called for %s", m_cf_prefix.c_str());
}

void LowLevelCommanderMinimal::notify_setpoints_stop(
    int remain_valid_milliseconds,
    double group_mask)
{
    auto req = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();
    req->remain_valid_millisecs = remain_valid_milliseconds;
    req->group_mask = group_mask;

    m_notify_setpoints_stop_client->async_send_request(req);
}

void LowLevelCommanderMinimal::cmd_position(
    const Eigen::Vector3d & position,
    double yaw)
{
    auto msg = crazyflie_interfaces::msg::Position();
    msg.x = position.x();
    msg.y = position.y();
    msg.z = position.z();
    msg.yaw = yaw;

    m_cmd_position_pub->publish(msg);
}

void LowLevelCommanderMinimal::cmd_velocity_world(
    const Eigen::Vector3d & linear_velocity,
    double yaw_rate)
{
    auto msg = crazyflie_interfaces::msg::VelocityWorld();
    msg.vel.x = linear_velocity.x();
    msg.vel.y = linear_velocity.y();
    msg.vel.z = linear_velocity.z();
    // Crazyflie expects yaw rate in degrees per second
    msg.yaw_rate = yaw_rate * 180.0 / 3.14159265358979323846;

    m_cmd_velocity_world_pub->publish(msg);
}
