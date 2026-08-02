#include "padflies_cpp/commander/commander.hpp"

PadflieCommander::PadflieCommander(
    const std::string & prefix,
    const std::string & cf_prefix,
    padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
)
: PadflieCommanderBase(prefix, cf_prefix, node_interfaces_bundle)
, m_node_base_interface(node_interfaces_bundle.base_interface)
, m_node_timers_interface(node_interfaces_bundle.timers_interface)
, m_node_clock_interface(node_interfaces_bundle.clock_interface)
, m_site_selector(std::make_shared<SiteSelector>(
    node_interfaces_bundle,
    m_padflie_tf,
    m_logger))
, m_routine_factory(std::make_shared<RoutineFactory>(
    node_interfaces_bundle,
    m_logger))
{
    using namespace padflies_cpp::commander;
    m_goal_executor = std::make_unique<RoutineFlightGoalExecutor>(
        m_routine_factory,
        m_site_selector,
        [this]() {
            return m_hw_state_controller.is_flying();
        },
        [this](FlightGoalKind goal_kind) {
            m_on_goal_started(goal_kind);
        },
        [this](FlightGoalKind goal_kind, GoalResult result) {
            m_on_goal_finished(goal_kind, std::move(result));
        }
    );
    m_goal_manager = std::make_unique<FlightGoalManager>(
        *m_goal_executor,
        std::make_shared<RclcppCommanderEventSink>(m_logger));
}

PadflieCommander::~PadflieCommander()
{
    if (m_goal_manager) {
        m_goal_manager->cancel_all_goals("Commander is being destroyed");
    }
}


bool 
PadflieCommander::is_healthy() const  
{
    bool isnt_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.is_flying();
    bool cant_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.canfly();
    return !isnt_flying && !cant_flying; 
}

bool 
PadflieCommander::get_home_state() const 
{
    return m_state != CommanderState::FLYING;
}

void 
PadflieCommander::m_configure_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
{
    m_create_availability_interface(node);
}

void PadflieCommander::m_on_commander_configured() 
{
    m_state = CommanderState::CONFIGURED;
}

void
PadflieCommander::m_activate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
{
    m_remove_availability_interface();
    m_create_goal_services(node);
}

void PadflieCommander::m_on_commander_activated() 
{
    m_routine_factory->set_padflie_shared_ptrs(m_hardware_actor, m_padflie_tf);
    m_state = m_hw_state_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
}

void 
PadflieCommander::m_deactivate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force) 
{    
    m_remove_goal_services();
    if (force) {
        m_goal_manager->cancel_all_goals("Commander was force-deactivated");
        m_create_availability_interface(node);
        return;
    }

    auto completion = std::make_shared<padflies_cpp::commander::BlockingGoalCompletion>();
    m_goal_manager->request_goal(
        padflies_cpp::commander::Return{},
        completion,
        padflies_cpp::commander::GoalPolicy::LOCKED);
    RCLCPP_INFO(m_logger, "Deactivating commander; returning padflie %s", m_cf_prefix.c_str());
    const auto result = completion->wait();
    if (result.outcome != padflies_cpp::commander::GoalOutcome::SUCCESS) {
        RCLCPP_WARN(m_logger, "Deactivation return failed: %s", result.message.c_str());
    }
    m_create_availability_interface(node);

}

void PadflieCommander::m_on_commander_deactivated() 
{
    m_routine_factory->reset_padflie_shared_ptrs();
    m_state = CommanderState::CONFIGURED;
}

void 
PadflieCommander::m_on_charged_callback() 
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}

void PadflieCommander::m_on_state_callback()
{
    if (!m_availability_pub ||
        !m_hw_state_controller.is_charged() ||
        !m_hw_state_controller.canfly() ||
        m_hw_state_controller.is_tumbled())
    {
        return;
    }

    Eigen::Vector3d position;
    if (!m_padflie_tf->get_cf_position(position)) return;

    const auto current_site = m_site_selector->get_current_site();
    if (!current_site) {
        RCLCPP_WARN(m_logger, "Cannot publish availability without current SiteInfo.");
        return;
    }

    padflies_interfaces::msg::AvailabilityInfo message;
    message.name = m_prefix;
    message.site_info = *current_site;
    m_availability_pub->publish(message);
}

void PadflieCommander::m_create_availability_interface(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
    auto options = rclcpp::PublisherOptions();
    options.callback_group = m_callback_group;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                .best_effort()
                .durability_volatile();
    m_availability_pub = node->create_publisher<padflies_interfaces::msg::AvailabilityInfo>(
        "availability", qos, options);
}

void PadflieCommander::m_remove_availability_interface()
{
    m_availability_pub.reset();
}

void PadflieCommander::m_create_goal_services(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
    m_deploy_to_service = node->create_service<padflies_interfaces::srv::DeployTo>(
        m_prefix + "/deploy_to",
        std::bind(
            &PadflieCommander::m_handle_deploy_to_goal, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        rclcpp::ServicesQoS(),
        m_callback_group);

    m_return_to_service = node->create_service<padflies_interfaces::srv::ReturnTo>(
        m_prefix + "/return_to",
        std::bind(
            &PadflieCommander::m_handle_return_to_goal, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        rclcpp::ServicesQoS(),
        m_callback_group);
}

void PadflieCommander::m_remove_goal_services()
{
    m_deploy_to_service.reset();
    m_return_to_service.reset();
}

void PadflieCommander::m_on_goal_started(
    padflies_cpp::commander::FlightGoalKind goal_kind)
{
    using padflies_cpp::commander::FlightGoalKind;
    const bool deploying =
        goal_kind == FlightGoalKind::DEPLOY ||
        goal_kind == FlightGoalKind::DEPLOY_TO;
    m_state = deploying ? CommanderState::TAKEOFF : CommanderState::LANDING;
}

void PadflieCommander::m_on_goal_finished(
    padflies_cpp::commander::FlightGoalKind goal_kind,
    padflies_cpp::commander::GoalResult result)
{
    using namespace padflies_cpp::commander;
    const bool deployed =
        goal_kind == FlightGoalKind::DEPLOY ||
        goal_kind == FlightGoalKind::DEPLOY_TO;

    if (result.outcome == GoalOutcome::SUCCESS) {
        m_state = deployed ? CommanderState::FLYING : CommanderState::CHARGING;
    } else if (m_hw_state_controller.is_flying()) {
        m_state = CommanderState::FLYING;
    } else {
        m_state = m_hw_state_controller.is_charged() ?
            CommanderState::CHARGED : CommanderState::CHARGING;
    }

    m_goal_manager->active_routine_finished(std::move(result));
}

void 
PadflieCommander::m_handle_takeoff_command(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req) 
{
    (void)req;
    using namespace padflies_cpp::commander;
    m_goal_manager->request_goal(
        Deploy{},
        std::make_shared<TriggerGoalCompletion>(service_handle, request_id));
}

void 
PadflieCommander::m_handle_land_command(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req) 
{   
    (void)req;
    using namespace padflies_cpp::commander;
    m_goal_manager->request_goal(
        Return{},
        std::make_shared<TriggerGoalCompletion>(service_handle, request_id));
}

void PadflieCommander::m_handle_deploy_to_goal(
    const std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::DeployTo>> service,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<padflies_interfaces::srv::DeployTo::Request> request)
{
    using namespace padflies_cpp::commander;
    m_goal_manager->request_goal(
        DeployTo{request->target},
        std::make_shared<ServiceGoalCompletion<padflies_interfaces::srv::DeployTo>>(
            service, request_id));
}

void PadflieCommander::m_handle_return_to_goal(
    const std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::ReturnTo>> service,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<padflies_interfaces::srv::ReturnTo::Request> request)
{
    using namespace padflies_cpp::commander;
    m_goal_manager->request_goal(
        ReturnTo{request->site},
        std::make_shared<ServiceGoalCompletion<padflies_interfaces::srv::ReturnTo>>(
            service, request_id));
}

void 
PadflieCommander::m_handle_send_target_command(
    const padflies_interfaces::msg::SendTarget::SharedPtr msg) 
{
    if (m_state == CommanderState::FLYING) {
        if (msg->mode_position) 
        {
            PoseTarget target;
            tf2::fromMsg(msg->target.pose, target.pose);
            target.frame_id = msg->target.header.frame_id;

            target.use_yaw = msg->use_yaw;
            target.collision_avoidance = msg->collision_avoidance;
            m_hardware_actor->set_pose_target(target);
        } else 
        {
            VelocityTarget target;
            tf2::fromMsg(msg->velocity.twist, target.velocity);
            target.frame_id = msg->velocity.header.frame_id;
            target.use_angular = msg->use_yaw_velocity;
            target.collision_avoidance = msg->collision_avoidance;
            m_hardware_actor->set_velocity_target(target);
        }
    } else {
        RCLCPP_DEBUG(m_logger, "Ignoring send_target command because padflie is not flying.");
    }
}
