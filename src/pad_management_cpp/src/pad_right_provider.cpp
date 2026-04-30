#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cstdio>
#include <string>
#include <unordered_map>

#include "pad_management_interfaces/action/pad_right_control.hpp"

#include <memory>
#include <thread>
#include <mutex>

struct  Request
{
  std::string name;
  rclcpp::Time request_time;
  rclcpp::Duration max_wait_time;
  rclcpp::Duration usage_time;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle;

  std::unique_lock<std::mutex> pad_lock; // Mutex to protect access to the pad for this request
  rclcpp::Time acquire_time; // Time when the pad was acquired
};

rclcpp::Duration duration_from_seconds(float seconds)
{
  int32_t sec = static_cast<int32_t>(seconds);
  uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
  return rclcpp::Duration(sec, nanosec);
}

class PadRightActionServer : public rclcpp::Node
{
public:
    PadRightActionServer() : Node("pad_right_action_server")
    {
      m_execution_timer = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_node_clock_interface()->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&PadRightActionServer::manage_requests, this)
      );

        m_action_server = rclcpp_action::create_server<pad_management_interfaces::action::PadRightControl>(
            this,
            "pad_right_control",
            std::bind(&PadRightActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PadRightActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PadRightActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

    rclcpp::Duration m_max_hold_time=duration_from_seconds(2.0);

    std::shared_ptr<rclcpp::TimerBase> m_execution_timer;
    std::shared_ptr<rclcpp_action::Server<pad_management_interfaces::action::PadRightControl>> m_action_server;
    
    std::mutex m_request_mutex;
    std::unordered_map<rclcpp_action::GoalUUID, Request> m_request_map;
  


    std::mutex m_pad_lock;


private: 
    void manage_requests()
    {
      std::lock_guard<std::mutex> lock(m_request_mutex);
      m_check_cancelations(m_request_map);
      m_check_timeouts(m_request_map);

      if (m_select_new_owner(m_request_map))
      {
        m_publish_feedback(m_request_map);      
      }
    }

    void m_publish_feedback(const std::unordered_map<rclcpp_action::GoalUUID, Request> & request_map)
    {
      auto feedback = std::make_shared<pad_management_interfaces::action::PadRightControl::Feedback>();

      for (const auto & pair : request_map) {
        const auto & request = pair.second;
        if (request.pad_lock.owns_lock()) {
          feedback->status = pad_management_interfaces::action::PadRightControl_Feedback::STATUS_ACQUIRED_RIGHT;
        } else {
          feedback->status = pad_management_interfaces::action::PadRightControl_Feedback::STATUS_WAITING_FOR_RIGHT;
        }
        request.goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback for goal %s: status %i", request.name.c_str(), feedback->status);
      }
    }


    bool m_select_new_owner(std::unordered_map<rclcpp_action::GoalUUID, Request> & request_map)
    {
      for (auto & pair : request_map) {   
        auto & request = pair.second;
        if (!request.pad_lock.owns_lock() && request.pad_lock.try_lock()) { // This entry gains ownership.
          request.acquire_time = this->get_node_clock_interface()->get_clock()->now();
          RCLCPP_INFO(this->get_logger(), "Goal %s has gained ownership of the pad", request.name.c_str());
          return true;
        }
      }
      return false;
    }

    void m_check_cancelations(std::unordered_map<rclcpp_action::GoalUUID, Request> & request_map)
    {
      for (auto it = request_map.begin(); it != request_map.end(); ) {
        auto & request = it->second;
        if (request.goal_handle->is_canceling())
        {
          bool was_owner = request.pad_lock.owns_lock();

          auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();
          result->success = true;
          request.goal_handle->canceled(result); 
          it = request_map.erase(it); // Erasing will clear the lock

          RCLCPP_INFO(this->get_logger(), "Goal %s is canceling, removing from request map, owner? %s", request.name.c_str(), was_owner ? "Yes" : "No");
        } else ++it;
      }
    }

    void m_check_timeouts(std::unordered_map<rclcpp_action::GoalUUID, Request> & request_map)
    {
      auto now = this->get_node_clock_interface()->get_clock()->now();
      for (auto it = request_map.begin(); it != request_map.end(); ) {
        auto & request = it->second;
        if (request.pad_lock.owns_lock() && now - request.acquire_time >= m_max_hold_time) {
          auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();
          result->success = false;
          request.goal_handle->abort(result); // Abort the goal
          it = request_map.erase(it); // Clears the lock automatically

          RCLCPP_INFO(this->get_logger(), "Pad released for goal %s due to hold time exceeded", request.name.c_str());
        } else ++it;
      }
    }
    

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal)
    {
      (void) uuid;
      RCLCPP_INFO(this->get_logger(), "Received goal request with name %s, will always accept.", goal->name.c_str());
      std::lock_guard<std::mutex> lock(m_request_mutex);
      {
        for (const auto & pair : m_request_map) {
          const auto & request = pair.second;
          if (request.name == goal->name) {
            RCLCPP_INFO(this->get_logger(), "Goal with name %s already exists, rejecting new goal.", goal->name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      auto name = goal_handle->get_goal()->name;
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for %s", name.c_str());
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      auto goal = goal_handle->get_goal();
      auto uuid = goal_handle->get_goal_id();

      Request request{
        .name = goal->name,
        .request_time = this->get_clock()->now(),
        .max_wait_time = duration_from_seconds(goal->max_wait_time),
        .usage_time = duration_from_seconds(goal->usage_time),
        .goal_handle = goal_handle,
        .pad_lock = std::unique_lock<std::mutex>(m_pad_lock, std::defer_lock),
        .acquire_time = rclcpp::Time(0, 0, RCL_ROS_TIME) // Initialize to zero time
      };
      
      std::lock_guard<std::mutex> lock(m_request_mutex);
      m_request_map.emplace(uuid, std::move(request));      
    }
};



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PadRightActionServer>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
