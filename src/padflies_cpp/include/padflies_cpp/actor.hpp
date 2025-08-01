#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "padflies_cpp/yaw_controller.hpp"
#include "padflies_cpp/position_controller.hpp"
#include "padflies_cpp/collision_avoidance_client.hpp"

#include "padflies_cpp/hl_commander_minimal.hpp"
#include "padflies_cpp/ll_commander_minimal.hpp"

#include "padflies_cpp/padflie_tf.hpp"

enum ActorState {
  DEACTIVATED,
  LOW_LEVEL_COMMANDER,
  HIGH_LEVEL_COMMANDER,
  ERROR_STATE
};

class PadflieActor
{
public:
    PadflieActor(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const std::string & cf_prefix,
        PadflieTF* padflie_tf
    );

    ~PadflieActor();

    void set_target(
        const geometry_msgs::msg::PoseStamped & target_pose,
        bool use_yaw);
    
    bool takeoff_routine(
        double takeoff_height = 1.0);
    
    bool land_routine();


    void get_target_pose(
        geometry_msgs::msg::PoseStamped & target_pose) const;

    double get_yaw() const;
private: 
    void m_send_target_callback();

    void m_fail_safe(std::string reason);
private: 
    ActorState m_state;
    double m_dt;
    double m_current_yaw;

    geometry_msgs::msg::PoseStamped m_target_pose;
    bool m_fixed_yaw;
    double m_fixed_yaw_target;
    
    bool m_last_target_valid;
    Eigen::Vector3d m_last_valid_target_position;
    double m_last_valid_target_yaw;

    YawController m_yaw_controller;
    PositionController m_position_controller;
    CollisionAvoidanceClient m_collision_avoidance_client;


    HighLevelCommanderMinimal m_hl_commander;
    LowLevelCommanderMinimal m_ll_commander;
    
    PadflieTF * m_padflie_tf;

    rclcpp::TimerBase::SharedPtr m_send_target_timer;
};