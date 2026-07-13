#include "padflies_cpp/hl_commander_minimal.hpp"


static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

HighLevelCommanderMinimal::HighLevelCommanderMinimal(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
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

    m_takeoff_client = rclcpp::create_client<crazyflie_interfaces::srv::Takeoff>(
        node_base_interface, 
        node_graph_interface, 
        node_services_interface, 
        cf_prefix + "/takeoff",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);

    m_land_client = rclcpp::create_client<crazyflie_interfaces::srv::Land>(
        node_base_interface, 
        node_graph_interface, 
        node_services_interface, 
        cf_prefix + "/land",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);

    m_go_to_client = rclcpp::create_client<crazyflie_interfaces::srv::GoTo>(
        node_base_interface, 
        node_graph_interface, 
        node_services_interface, 
        cf_prefix + "/go_to",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);

}

HighLevelCommanderMinimal::~HighLevelCommanderMinimal()
{
    m_takeoff_client.reset();
    m_land_client.reset();
    m_go_to_client.reset();
    // m_callback_group.reset(); // See note above about m_callback_group
    RCLCPP_DEBUG(m_logger, "HighLevelCommanderMinimal destructor called for %s", m_cf_prefix.c_str());
}

bool HighLevelCommanderMinimal::takeoff(
    double height,
    double duration_seconds,
    double yaw_rad,
    double group_mask)
{
    if (!m_takeoff_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_logger, "Service /%s/takeoff not available", m_cf_prefix.c_str());
        return false;
    }
    auto req = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
    req->height = height;
    req->duration = rclcpp::Duration::from_seconds(duration_seconds);
    req->yaw = yaw_rad;
    req->group_mask = group_mask;

    m_takeoff_client->async_send_request(req);
    return true;
}

bool HighLevelCommanderMinimal::land(
    double target_height,
    double duration_seconds,
    double yaw_rad,
    double group_mask)
{
    if (!m_land_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_logger, "Service /%s/land not available", m_cf_prefix.c_str());
        return false;
    }
    auto req = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
    req->height = target_height;
    req->duration = rclcpp::Duration::from_seconds(duration_seconds);
    req->yaw = yaw_rad;
    req->group_mask = group_mask;

    m_land_client->async_send_request(req);
    return true;
}

bool HighLevelCommanderMinimal::go_to(
    const Eigen::Vector3d & position,
    double yaw_rad,
    double duration_seconds,
    bool relative,
    double group_mask)
{
    if (!m_go_to_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_logger, "Service /%s/go_to not available", m_cf_prefix.c_str());
        return false;
    }
    auto req = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
    req->goal.x = position.x();
    req->goal.y = position.y();
    req->goal.z = position.z();
    req->yaw = yaw_rad;
    req->duration = rclcpp::Duration::from_seconds(duration_seconds);
    req->relative = relative;
    req->group_mask = group_mask;

    m_go_to_client->async_send_request(req);
    return true;
}