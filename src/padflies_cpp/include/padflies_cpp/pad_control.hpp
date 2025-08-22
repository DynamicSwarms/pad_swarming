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

    void set_node(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    void activate(
        const std::string & pad_name,
        const std::string & prefix);
    
    void deactivate();

    void acquire_right_async(double timeout_seconds, RightCallbackT && callback);
    // takeoff true land false
    void release_right_async(bool takeoff_land, RightCallbackT && callback);

    bool get_pad_circle_target(
        double timeout_seconds,
        const geometry_msgs::msg::PoseStamped & position, 
        geometry_msgs::msg::PoseStamped & target_position);

private: 
    std::string m_prefix;

    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> m_node;

    rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedPtr m_acquire_client;
    rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedPtr m_release_client;
    rclcpp::Client<pad_management_interfaces::srv::PadIdleTarget>::SharedPtr m_pad_idle_target_client;

    std::string m_logger_name;
};