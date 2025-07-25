#include "padflies_cpp/ll_commander_minimal.hpp"

static rclcpp::CallbackGroup::SharedPtr m_callback_group = nullptr; 
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

LowLevelCommanderMinimal::LowLevelCommanderMinimal(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix)
: m_cf_prefix(cf_prefix)
{
    if (!m_callback_group)
        m_callback_group = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    //m_callback_group = node->create_callback_group(
    //    rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;

    m_notify_setpoints_stop_pub = node->create_publisher<crazyflie_interfaces::msg::NotifySetpointsStop>(
         m_cf_prefix + "/notify_setpoints_stop",10, pub_options);
    m_cmd_position_pub = node->create_publisher<crazyflie_interfaces::msg::Position>(
        m_cf_prefix + "/cmd_position", 10, pub_options);
}

LowLevelCommanderMinimal::~LowLevelCommanderMinimal()
{
    m_notify_setpoints_stop_pub.reset();
    m_cmd_position_pub.reset();
    // m_callback_group.reset(); // See note above about m_callback_group
    RCLCPP_INFO(rclcpp::get_logger("LowLevelCommanderMinimal"), "LowLevelCommanderMinimal destructor called for %s", m_cf_prefix.c_str());
}

void LowLevelCommanderMinimal::notify_setpoints_stop(
    int remain_valid_milliseconds,
    double group_mask)
{
    auto msg = crazyflie_interfaces::msg::NotifySetpointsStop();
    msg.remain_valid_millisecs = remain_valid_milliseconds;
    msg.group_mask = group_mask;

    m_notify_setpoints_stop_pub->publish(msg);
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