#include "padflies_cpp/ll_commander_minimal.hpp"



LowLevelCommanderMinimal::LowLevelCommanderMinimal(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix)
: m_cf_prefix(cf_prefix)
{
    //callback_group = node->create_callback_group(
    //    rclcpp::CallbackGroupType::MutuallyExclusive);

    //auto pub_options = rclcpp::PublisherOptions();
    //pub_options.callback_group = callback_group;

    m_notify_setpoints_stop_pub = node->create_publisher<crazyflie_interfaces::msg::NotifySetpointsStop>(
         m_cf_prefix + "/notify_setpoints_stop",10); //, 10, pub_options);
    // m_cmd_position_pub = node->create_publisher<crazyflie_interfaces::msg::Position>(
    //     m_cf_prefix + "/cmd_position", 10, pub_options);
}

LowLevelCommanderMinimal::~LowLevelCommanderMinimal()
{
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