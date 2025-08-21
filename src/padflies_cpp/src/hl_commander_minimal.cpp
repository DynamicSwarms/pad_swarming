#include "padflies_cpp/hl_commander_minimal.hpp"


static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

HighLevelCommanderMinimal::HighLevelCommanderMinimal(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix)
: m_cf_prefix(cf_prefix)
, m_logger_name(node->get_name())
{
     if (m_callback_groups.find(cf_prefix) == m_callback_groups.end())
        m_callback_groups[cf_prefix] = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_groups[cf_prefix];

    m_takeoff_pub = node->create_publisher<crazyflie_interfaces::msg::Takeoff>(
        m_cf_prefix + "/takeoff", 10, pub_options);

    m_land_pub = node->create_publisher<crazyflie_interfaces::msg::Land>(
        m_cf_prefix + "/land", 10, pub_options);

    m_go_to_pub = node->create_publisher<crazyflie_interfaces::msg::GoTo>(
        m_cf_prefix + "/go_to", 10, pub_options);
}

HighLevelCommanderMinimal::~HighLevelCommanderMinimal()
{
    m_takeoff_pub.reset();
    m_land_pub.reset();
    m_go_to_pub.reset();
    // m_callback_group.reset(); // See note above about m_callback_group
    RCLCPP_DEBUG(rclcpp::get_logger(m_logger_name), "HighLevelCommanderMinimal destructor called for %s", m_cf_prefix.c_str());
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