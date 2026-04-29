#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cstdio>

#include "pad_management_interfaces/action/pad_right_control.hpp"

class PadRightActionServer : public rclcpp::Node
{
public:
    PadRightActionServer() : Node("pad_right_action_server")
    {
        m_action_server = rclcpp_action::create_server<pad_management_interfaces::action::PadRightControl>(
            this,
            "pad_right_control",
            std::bind(&PadRightActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PadRightActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PadRightActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }
    std::shared_ptr<rclcpp_action::Server<pad_management_interfaces::action::PadRightControl>> m_action_server;

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with name %s", goal->name.c_str());
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      std::thread{std::bind(&PadRightActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(1);
      auto feedback = std::make_shared<pad_management_interfaces::action::PadRightControl::Feedback>();
      auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();

      for (int i = 0; i < 5; ++i) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        feedback->status = i;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback: %i", feedback->status);
        loop_rate.sleep();
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
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
