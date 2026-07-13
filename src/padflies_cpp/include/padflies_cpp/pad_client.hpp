#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"
#include "pad_management_interfaces/srv/pad_idle_target.hpp"
#include "padflies_cpp/padflie_tf.hpp"

struct PadRightRequest
{
    uint8_t action;
    rclcpp::Duration max_wait_time{rclcpp::Duration::from_seconds(0.0)};
    rclcpp::Duration usage_time{rclcpp::Duration::from_seconds(0.0)};
    double battery_percentage;
    geometry_msgs::msg::PoseStamped pose;
};

class PadClient
{
public:
    using PadRightControlActionT = pad_management_interfaces::action::PadRightControl;
    using PadRightControlGoalHandleT = rclcpp_action::ClientGoalHandle<PadRightControlActionT>;

    PadClient(
        const std::string & prefix,
        const std::string & pad_name,
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group,
        rclcpp::Logger parent_logger)
        : m_prefix(prefix)
        , m_pad_name(pad_name)
        , m_logger(parent_logger.get_child("PadClient[" + pad_name + "]"))
        , m_padflie_tf(padflie_tf)
    {
        m_pad_right_control_action_client = rclcpp_action::create_client<PadRightControlActionT>(
                node_base_interface,
                node_graph_interface,
                node_logging_interface,
                node_waitables_interface,
                pad_name + "/pad_right_control",
                callback_group);

        m_pad_idle_target_client = rclcpp::create_client<pad_management_interfaces::srv::PadIdleTarget>(
                node_base_interface,
                node_graph_interface,
                node_services_interface,
                pad_name + "/pad_idle_target",
                rclcpp::ServicesQoS().keep_last(10),
                callback_group);
    }

    bool wait_for_action_server(std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return m_pad_right_control_action_client->wait_for_action_server(timeout);
    }

    bool is_action_server_available() {
        return m_pad_right_control_action_client->action_server_is_ready();
    }

    bool get_pad_idle_target(
        double timeout_seconds,
        const Eigen::Affine3d & position,
        const std::string & frame_id,
        Eigen::Affine3d & target_position,
        std::string & target_frame_id)
    {
        auto request = std::make_shared<pad_management_interfaces::srv::PadIdleTarget::Request>();
        request->name = m_prefix;
        request->position = geometry_msgs::msg::PoseStamped();
        // TODO: clock now
        request->position.header.frame_id = frame_id;
        request->position.pose.position.x = position.translation().x();
        request->position.pose.position.y = position.translation().y();
        request->position.pose.position.z = position.translation().z();
        request->position.pose.orientation.x = 0.0;
        request->position.pose.orientation.y = 0.0;
        request->position.pose.orientation.z = 0.0;
        request->position.pose.orientation.w = 1.0;

        if (m_pad_idle_target_client->wait_for_service(std::chrono::milliseconds(static_cast<int>(timeout_seconds * 1000)))) {
            auto result_future = m_pad_idle_target_client->async_send_request(request);
            if (result_future.wait_for(std::chrono::milliseconds(static_cast<int>(timeout_seconds * 1000))) == std::future_status::ready) {
                auto response = result_future.get();
                target_position.translation().x() = response->target.pose.position.x;
                target_position.translation().y() = response->target.pose.position.y;
                target_position.translation().z() = response->target.pose.position.z;
                target_position.linear() = Eigen::Quaterniond(
                    response->target.pose.orientation.w,
                    response->target.pose.orientation.x,
                    response->target.pose.orientation.y,
                    response->target.pose.orientation.z).toRotationMatrix();
                target_frame_id = response->target.header.frame_id;
                return true;
            } else {
                RCLCPP_ERROR(m_logger, "Service /%s/pad_idle_target did not respond after waiting for %f seconds", m_pad_name.c_str(), timeout_seconds);
                return false;
            }
        } else {
            RCLCPP_ERROR(m_logger, "Service /%s/pad_idle_target not available after waiting for %f seconds", m_pad_name.c_str(), timeout_seconds);
            return false;
        }

        RCLCPP_INFO(m_logger, "Whaat, not possible");
        return false;
    }

    void send_request(PadRightRequest request) 
    {
        auto goal_msg = PadRightControlActionT::Goal();
        goal_msg.action = request.action;
        goal_msg.max_wait_time = request.max_wait_time;
        goal_msg.usage_time = request.usage_time;
        goal_msg.battery_percentage = request.battery_percentage;
        goal_msg.current_pose = request.pose;

        goal_msg.name = m_prefix;
        auto send_goal_options = rclcpp_action::Client<PadRightControlActionT>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PadClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&PadClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&PadClient::result_callback, this, std::placeholders::_1);
        m_pad_right_control_action_client->async_send_goal(goal_msg, send_goal_options);
    }

    void cancel_goal() 
    {
        if (m_current_goal_handle) {
            m_pad_right_control_action_client->async_cancel_goal(m_current_goal_handle);
        } else {
            RCLCPP_WARN(m_logger, "No current goal to cancel");
        }
    }

    geometry_msgs::msg::PoseStamped get_target_pose() const { return m_target_pose; }

    std::string get_pad_name() const { return m_pad_name; }

    bool goal_responded() { return m_goal_responded; }
    bool goal_accepted() { return m_goal_accepted; }

    bool has_right(){return m_right_acquired;}

    bool received_result() { return m_received_result; }
    bool result_success() { return m_result_success; }

private:
    void goal_response_callback(const typename PadRightControlGoalHandleT::SharedPtr & goal_handle) 
    {
        m_current_goal_handle = goal_handle;
        m_goal_accepted = !!goal_handle;
        m_goal_responded = true;
        
        if (!goal_handle) {
            RCLCPP_ERROR(m_logger, "Goal rejected");
        } else {
            RCLCPP_INFO(m_logger, "Goal accepted");
        }
    }

    void feedback_callback(
        typename PadRightControlGoalHandleT::SharedPtr goal_handle,
        const std::shared_ptr<const PadRightControlActionT::Feedback> feedback)
    {
        (void)goal_handle;
        m_right_acquired = feedback->status == PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT;
        m_target_pose = feedback->target_pose;

        if (feedback->status == PadRightControlActionT::Feedback::STATUS_WAITING_FOR_RIGHT)
            RCLCPP_INFO(m_logger, "FBD: Pad says PadClient needs to wait for right...");
        else if (feedback->status == PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT)
            RCLCPP_INFO(m_logger, "FBD: Pad says PadClient acquired right!");
    }

    void result_callback(const typename PadRightControlGoalHandleT::WrappedResult & result) 
    {
        m_result_success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        m_received_result = true;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Pad right control action finished with: SUCCESS");
        } else {
            RCLCPP_ERROR(m_logger, "Pad right control action failed with: FAILED");
        }
    }



private: 
    std::string m_prefix;
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<PadflieTF> m_padflie_tf;


    std::shared_ptr<rclcpp_action::Client<PadRightControlActionT>> m_pad_right_control_action_client;
    std::shared_ptr<rclcpp::Client<pad_management_interfaces::srv::PadIdleTarget>> m_pad_idle_target_client;

    geometry_msgs::msg::PoseStamped m_target_pose; // Land/Takeoff pose received from pad

private: 
    std::shared_ptr<PadRightControlGoalHandleT> m_current_goal_handle;
    bool m_goal_responded = false;
    bool m_goal_accepted = false;
    bool m_right_acquired = false;
    
    bool m_received_result = false;
    bool m_result_success = false;
};
    