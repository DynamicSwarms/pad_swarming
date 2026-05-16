#include "rqt_padflies/padflie_lifecycle_connection.hpp"

PadflieLifecycleConnection::PadflieLifecycleConnection(
    std::string prefix, 
    LifecycleStateCallback lifecycle_state_callback,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : m_lifecycle_state_callback(lifecycle_state_callback)
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
    m_get_state_client = rclcpp::create_client<lifecycle_msgs::srv::GetState>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        prefix + "/get_state",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        callback_group);
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

                callback(response->success);
            }
            catch (const std::exception & e) {
                callback(false);
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

                m_lifecycle_state_callback(response->current_state);
            }
            catch (const std::exception & e) {
                std::cerr << "Failed to get current lifecycle state: " << e.what() << std::endl;
            }
        };
    m_get_state_client->async_send_request(request, response_callback);
}

void 
PadflieLifecycleConnection::activate_padflie_with_callback(std::function<void(bool)> callback)
{
    transition_padflie_with_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, "activate", callback);
}

void 
PadflieLifecycleConnection::deactivate_padflie_with_callback(std::function<void(bool)> callback)
{
    transition_padflie_with_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, "deactivate", callback);
}

void 
PadflieLifecycleConnection::m_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    m_lifecycle_state_callback(msg->goal_state);
}
