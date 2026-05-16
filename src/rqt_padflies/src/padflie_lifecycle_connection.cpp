#include "rqt_padflies/padflie_lifecycle_connection.hpp"

PadflieLifecycleConnection::PadflieLifecycleConnection(
    std::string prefix, 
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group,
    rclcpp::Logger logger)
    : m_lifecycle_state_callback(nullptr)
    , m_logger(logger)
{
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = callback_group;

    m_transition_event_sub = rclcpp::create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        node_topics_interface,
        prefix + "/transition_event",
        rclcpp::QoS(10),
        std::bind(&PadflieLifecycleConnection::m_transition_event_callback, this, std::placeholders::_1),
        subscription_options);

    m_change_state_client = rclcpp::create_client<lifecycle_msgs::srv::ChangeState>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        prefix + "/change_state",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        callback_group);

    m_callback_group = node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_get_state_client = rclcpp::create_client<lifecycle_msgs::srv::GetState>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        prefix + "/get_state",
        rmw_qos_profile_services_default,
        m_callback_group);
}

PadflieLifecycleConnection::~PadflieLifecycleConnection()
{
    m_transition_event_sub.reset();
    m_change_state_client.reset();
    m_get_state_client.reset();
}

void
PadflieLifecycleConnection::transition_padflie_with_callback(
    uint8_t id,
    const std::string &label,
    std::function<void(bool)> callback)
{
    if (!m_change_state_client->service_is_ready()) return;

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    request->transition.label = label;

    auto response_callback =
        [callback](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
        {
            try {
                auto response = future.get();

                if (callback) callback(response->success);
            }
            catch (const std::exception & e) {
                if (callback) callback(false);
            }
        };
    m_change_state_client->async_send_request(request, response_callback);
}

void
PadflieLifecycleConnection::poll_current_lifecycle_state()
{
    if (!m_get_state_client->service_is_ready()) return;
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto response_callback =
        [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
        {
            try {
                auto response = future.get();

                if (m_lifecycle_state_callback) {
                    m_lifecycle_state_callback(response->current_state);
                }
            }
            catch (const std::exception & e) {
                RCLCPP_ERROR(m_logger, "Failed to get current lifecycle state: %s", e.what());
            }
        };
    m_get_state_client->async_send_request(request, response_callback);
}

void 
PadflieLifecycleConnection::activate_padflie()
{
    transition_padflie_with_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, "activate", nullptr);
}

void 
PadflieLifecycleConnection::deactivate_padflie()
{
    transition_padflie_with_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, "deactivate", nullptr);
}

void 
PadflieLifecycleConnection::m_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    if (m_lifecycle_state_callback) {
        m_lifecycle_state_callback(msg->goal_state);
    }
}
