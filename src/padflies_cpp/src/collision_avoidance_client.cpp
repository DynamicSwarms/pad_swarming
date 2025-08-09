#include "padflies_cpp/collision_avoidance_client.hpp"

static std::unordered_map<uint8_t, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

CollisionAvoidanceClient::CollisionAvoidanceClient(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    uint8_t cf_id)
: m_cf_id(cf_id)
, m_logger_name(node->get_name())
{
    if (m_callback_groups.find(cf_id) == m_callback_groups.end())
        m_callback_groups[cf_id] = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    
    m_client = node->create_client<collision_avoidance_interfaces::srv::CollisionAvoidance>(
        "collision_avoidance",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[cf_id]);
}

CollisionAvoidanceClient::~CollisionAvoidanceClient()
{
    m_client.reset();
    // m_callback_group.reset(); // See note above about m_callback_group
    RCLCPP_DEBUG(rclcpp::get_logger(m_logger_name), "CollisionAvoidanceClient destructor called");
}

void CollisionAvoidanceClient::get_collision_avoidance_target(
    const Eigen::Vector3d & position,
    Eigen::Vector3d & target)
{
    if (!m_client) return;

    auto request = std::make_shared<collision_avoidance_interfaces::srv::CollisionAvoidance::Request>();
    request->id = m_cf_id;
    request->position.x = position.x();
    request->position.y = position.y();
    request->position.z = position.z();

    request->target.x = target.x();
    request->target.y = target.y();
    request->target.z = target.z();

    auto result = m_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        target.x() = response->target.x;
        target.y() = response->target.y;
        target.z() = response->target.z;
    }
}
