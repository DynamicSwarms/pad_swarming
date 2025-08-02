#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pad_management_interfaces/srv/pad_right_acquire.hpp"
#include "pad_management_interfaces/srv/pad_right_release.hpp"

#include "pad_management_interfaces/srv/pad_circle_behaviour.hpp"

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
        Eigen::Vector3d & position, 
        Eigen::Vector3d & target_position);

    std::string get_frame_name() const;

private: 
    std::string m_prefix;
    std::string m_frame_name;

    rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedPtr m_acquire_client;
    rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedPtr m_release_client;
    rclcpp::Client<pad_management_interfaces::srv::PadCircleBehaviour>::SharedPtr m_circle_behaviour_client;
};