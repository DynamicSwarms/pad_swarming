#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pad_management_interfaces/srv/pad_right_acquire.hpp"
#include "pad_management_interfaces/srv/pad_right_release.hpp"

#include "pad_management_interfaces/srv/pad_circle_behaviour.hpp"

#include <Eigen/Dense>

class PadControl
{
public:
    PadControl();

    void configure(const std::string & cf_prefix,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
    
    void unconfigure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    bool acquire_right(double timeout_seconds);
    bool release_right();

    bool get_pad_circle_target(
        double timeout_seconds,
        Eigen::Vector3d & position, 
        Eigen::Vector3d & target_position);

private: 
    std::string m_cf_prefix;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedPtr m_acquire_client;
    rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedPtr m_release_client;
    rclcpp::Client<pad_management_interfaces::srv::PadCircleBehaviour>::SharedPtr m_circle_behaviour_client;
};