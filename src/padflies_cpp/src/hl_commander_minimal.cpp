#include "padflies_cpp/hl_commander_minimal.hpp"



HighLevelCommanderMinimal::HighLevelCommanderMinimal(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix)
: m_cf_prefix(cf_prefix)
{
    //callback_group = node->create_callback_group(
    //    rclcpp::CallbackGroupType::MutuallyExclusive);

    //auto pub_options = rclcpp::PublisherOptions();
    //pub_options.callback_group = callback_group;

    //m_takeoff_pub = node->create_publisher<crazyflie_interfaces::msg::Takeoff>(
    //    m_cf_prefix + "/takeoff", 10, pub_options);
//
    //m_land_pub = node->create_publisher<crazyflie_interfaces::msg::Land>(
    //    m_cf_prefix + "/land", 10, pub_options);
//
    //m_go_to_pub = node->create_publisher<crazyflie_interfaces::msg::GoTo>(
    //    m_cf_prefix + "/go_to", 10, pub_options);
}

HighLevelCommanderMinimal::~HighLevelCommanderMinimal()
{
    RCLCPP_INFO(rclcpp::get_logger("HighLevelCommanderMinimal"), "HighLevelCommanderMinimal destructor called for %s", m_cf_prefix.c_str());
}

void HighLevelCommanderMinimal::takeoff(
    double height,
    double duration_seconds,
    double yaw,
    bool use_current_yaw,
    double group_mask)
{
    auto msg = crazyflie_interfaces::msg::Takeoff();
    msg.height = height;
    msg.duration = rclcpp::Duration::from_seconds(duration_seconds);
    msg.yaw = yaw;
    msg.use_current_yaw = use_current_yaw;
    msg.group_mask = group_mask;

    m_takeoff_pub->publish(msg);
}

void HighLevelCommanderMinimal::land(
    double target_height,
    double duration_seconds,
    double yaw,
    bool use_current_yaw,
    double group_mask)
{
    auto msg = crazyflie_interfaces::msg::Land();
    msg.height = target_height;
    msg.duration = rclcpp::Duration::from_seconds(duration_seconds);
    msg.yaw = yaw;
    msg.use_current_yaw = use_current_yaw;
    msg.group_mask = group_mask;

    m_land_pub->publish(msg);
}

void HighLevelCommanderMinimal::go_to(
    const Eigen::Vector3d & position,
    double yaw,
    double duration_seconds,
    bool relative,
    bool linear,
    double group_mask)
{
    auto msg = crazyflie_interfaces::msg::GoTo();
    msg.goal.x = position.x();
    msg.goal.y = position.y();
    msg.goal.z = position.z();
    msg.yaw = yaw;
    msg.duration = rclcpp::Duration::from_seconds(duration_seconds);
    msg.relative = relative;
    msg.linear = linear;
    msg.group_mask = group_mask;

    m_go_to_pub->publish(msg);
}