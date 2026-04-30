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
  


    std::mutex m_mutex;


private: 
    void manage_requests()
    {
      std::lock_guard<std::mutex> lock(m_request_mutex);
      bool new_owner = false;
      for (auto & pair : m_request_map) {   
        auto & request = pair.second;
        if (request.goal_handle->is_canceling())
        {
          if (request.pad_lock.owns_lock()) {
            request.pad_lock.unlock(); // Release the pad if this goal is currently owning it
            RCLCPP_INFO(this->get_logger(), "Pad released for goal %s due to cancelation", request.name.c_str());
          }

          auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();
          result->success = true;
          request.goal_handle->canceled(result); // Mark the goal as canceled

          m_request_map.erase(pair.first); // Remove the request from the map
          return;
        }


        if (request.pad_lock.owns_lock()) {
          auto now = this->get_node_clock_interface()->get_clock()->now();
          if (now - request.acquire_time >= m_max_hold_time) {
            RCLCPP_INFO(this->get_logger(), "Pad released for goal %s due to hold time exceeded", request.name.c_str());


            request.pad_lock.unlock(); // Release the pad if the hold time has been exceeded
            auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();
            result->success = false;
            request.goal_handle->abort(result); // Abort the goal
            m_request_map.erase(pair.first); // Remove the request from the map
          }
          return;
        }


        if (request.pad_lock.try_lock()) { // This entry gains ownership.
          new_owner = true;
          request.acquire_time = this->get_node_clock_interface()->get_clock()->now();
          RCLCPP_INFO(this->get_logger(), "Goal %s has gained ownership of the pad", request.name.c_str());
          break;
        }
      }

      if (new_owner)
      {
        auto feedback = std::make_shared<pad_management_interfaces::action::PadRightControl::Feedback>();

        for (auto & pair : m_request_map) {
          auto & request = pair.second;
          if (request.pad_lock.owns_lock()) {
            feedback->status = pad_management_interfaces::action::PadRightControl_Feedback::STATUS_ACQUIRED_RIGHT;
          } else {
            feedback->status = pad_management_interfaces::action::PadRightControl_Feedback::STATUS_WAITING_FOR_RIGHT;
          }
          request.goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publishing feedback for goal %s: status %i", request.name.c_str(), feedback->status);
        }
      }
    }
    

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with name %s, will always accept.", goal->name.c_str());
      // Might reject here?
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      auto name = goal_handle->get_goal()->name;
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for %s", name.c_str());
      
      //rclcpp_action::GoalUUID uuid = goal_handle->get_goal_id();
      //{
      //  std::lock_guard<std::mutex> lock(m_request_mutex);
      //  auto it = m_request_map.find(uuid);
      //  if (it != m_request_map.end()) {
      //    auto & request = it->second;
      //    if (request.pad_lock.owns_lock()) {
      //      request.pad_lock.unlock(); // Release the pad if this goal is currently owning it
      //      RCLCPP_INFO(this->get_logger(), "Pad released for goal %s due to cancelation", request.name.c_str());
      //    }
      //    m_request_map.erase(it); // Remove the request from the map
      //  }
      //}


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
        .pad_lock = std::unique_lock<std::mutex>(m_mutex, std::defer_lock)
      };
      {
        std::lock_guard<std::mutex> lock(m_request_mutex);
        m_request_map.emplace(uuid, std::move(request));      
      }
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
