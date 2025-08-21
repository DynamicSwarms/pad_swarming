#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pad_management_interfaces/srv/pad_right_acquire.hpp"
#include "pad_management_interfaces/srv/pad_right_release.hpp"

#include "pad_management_interfaces/srv/pad_idle_target.hpp"

#include <Eigen/Dense>


class PadControl
{
public:
    using RightCallbackT = std::function<void(bool)>;
    

    PadControl();

    void on_activate(const std::string & prefix,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
    
    void on_deactivate(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    bool acquire_right(double timeout_seconds);
    bool release_right();

    void acquire_right_async(double timeout_seconds, RightCallbackT && callback);
    void release_right_async(RightCallbackT && callback);

    bool get_pad_circle_target(
        double timeout_seconds,
        const geometry_msgs::msg::PoseStamped & position, 
        geometry_msgs::msg::PoseStamped & target_position);

private: 
    std::string m_prefix;

    rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedPtr m_acquire_client;
    rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedPtr m_release_client;
    rclcpp::Client<pad_management_interfaces::srv::PadIdleTarget>::SharedPtr m_pad_idle_target_client;

    std::string m_logger_name;
};