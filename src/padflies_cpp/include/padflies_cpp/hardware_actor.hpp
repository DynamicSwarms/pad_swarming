#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "padflies_cpp/hardware_parameter_controller.hpp"

#include "padflies_cpp/yaw_controller.hpp"
#include "padflies_cpp/position_controller.hpp"

#include "padflies_cpp/hl_commander_minimal.hpp"
#include "padflies_cpp/ll_commander_minimal.hpp"

#include "padflies_cpp/padflie_tf.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
enum ActorState {
  DEACTIVATED,
  LOW_LEVEL_COMMANDER,
  HIGH_LEVEL_COMMANDER,
  ERROR_STATE
};

enum ActorMode {
    NONE, 
    POSITION_CONTROL,
    VELOCITY_CONTROL
};

struct EigenVelocityStamped
{
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    std::string frame_id;
};

struct PoseTarget
{
    Eigen::Affine3d pose;
    std::string frame_id;
    bool use_yaw;
    bool collision_avoidance;
};

class CollisionAvoidanceClient;

class HardwareActor
{
public:
    HardwareActor(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        const std::string & cf_prefix,
        std::shared_ptr<PadflieTF> padflie_tf
    );

    ~HardwareActor();
    std::string get_current_target_frame() const;

    bool set_pose_target(const PoseTarget & target_pose);

    bool set_velocity_target(
        const EigenVelocityStamped & velocity, 
        bool use_angular = true);

    bool go_to(
        const Eigen::Affine3d & target_pose,
        double duration,
        bool relative);

    bool land(
        double height, 
        double yaw_rad, 
        double duration);

    bool takeoff(
        double height, 
        double yaw_rad, 
        double duration
    );

    void reset_kalman_to(Eigen::Affine3d & pose);

    void fail_safe(std::string reason);
   
public:

    ActorMode get_mode() const { return m_mode; }

    void get_target_pose(
        PoseTarget & target_pose) const {
            target_pose = m_target_pose;
        };

    void get_target_velocity(
        EigenVelocityStamped & target_velocity) const {
            target_velocity = m_target_velocity;
        };


//    double get_yaw() const; // better: set current yaw into the padflie_tf


private: 
    void m_ll_command_timer_callback();

    void m_transition_to_low_level_commander();
    void m_transition_to_high_level_commander();

    void unpack_pose_target(
        const PoseTarget & pose_target,
        geometry_msgs::msg::PoseStamped & msg_pose_stamped,
        bool & use_yaw,
        bool & collision_avoidance) const
    {
        use_yaw = pose_target.use_yaw;
        collision_avoidance = pose_target.collision_avoidance;

        msg_pose_stamped.pose = tf2::toMsg(pose_target.pose);
        msg_pose_stamped.header.frame_id = pose_target.frame_id;
    }

private: 
    ActorState m_state;
    ActorMode m_mode;
    double m_dt;
    double m_current_yaw = 0.0;

private: // Targets 
    PoseTarget m_target_pose;
    bool m_fixed_yaw;

    EigenVelocityStamped m_target_velocity;
    bool m_use_angular_velocity;

    double m_fixed_yaw_target = 0.0;

    
    
private: // Internal state for ll_commander_callback 
    bool m_last_target_valid;
    Eigen::Vector3d m_last_valid_target_position;
    double m_last_valid_target_yaw;

    YawController m_yaw_controller;
    PositionController m_position_controller;
    std::unique_ptr<CollisionAvoidanceClient> m_collision_avoidance_client;

    std::shared_ptr<HardwareParameterController> m_hardware_parameter_controller;

    HighLevelCommanderMinimal m_hl_commander;
    LowLevelCommanderMinimal m_ll_commander;
    
    std::shared_ptr<PadflieTF> m_padflie_tf;

    rclcpp::TimerBase::SharedPtr m_send_target_timer;

    rclcpp::Logger m_logger;
};