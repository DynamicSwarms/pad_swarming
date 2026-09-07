#include "padflies_cpp/commander/commander.hpp"

#include <chrono>
#include <vector>

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
            return m_hw_state_controller->is_flying();
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
    bool isnt_flying = m_state == CommanderState::FLYING && !m_hw_state_controller->is_flying();
    bool cant_flying = m_state == CommanderState::FLYING && !m_hw_state_controller->canfly();
    return !isnt_flying && !cant_flying; 
}

bool 
PadflieCommander::get_home_state() const 
{
    return m_state != CommanderState::FLYING;
}

void 
PadflieCommander::m_configure_commander()
{
    m_create_availability_interface();
}

void PadflieCommander::m_on_commander_configured() 
{
    m_state = CommanderState::CONFIGURED;
}

void
PadflieCommander::m_activate_commander()
{
    m_remove_availability_interface();
    m_create_goal_interfaces();
}

void PadflieCommander::m_on_commander_activated() 
{
    m_routine_factory->set_padflie_shared_ptrs(
        m_hardware_actor, m_hw_state_controller, m_padflie_tf);
    m_state = m_hw_state_controller->is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
}

void 
PadflieCommander::m_deactivate_commander(
    bool force) 
{
    m_accepting_goals = false;
    if (force) {
        m_goal_manager->cancel_all_goals("Commander was force-deactivated");
    } else {
        auto completion = std::make_shared<padflies_cpp::commander::BlockingGoalCompletion>();
        m_goal_manager->request_goal(
            padflies_cpp::commander::Return{},
            completion,
            padflies_cpp::commander::GoalRetryPolicy::INFINITE,
            padflies_cpp::commander::GoalPolicy::LOCKED);
        RCLCPP_INFO(m_logger, "Deactivating commander; returning padflie %s", m_cf_prefix.c_str());
        const auto result = completion->wait();
        if (result.outcome != padflies_cpp::commander::GoalOutcome::SUCCESS) {
            RCLCPP_WARN(m_logger, "Deactivation return failed: %s", result.message.c_str());
        }
    }

    m_remove_goal_interfaces();
    m_create_availability_interface();
}

void PadflieCommander::m_on_commander_deactivated() 
{
    m_routine_factory->reset_padflie_shared_ptrs();
    m_state = CommanderState::CONFIGURED;
}

void PadflieCommander::m_cleanup_commander()
{
    m_remove_availability_interface();
    m_site_selector->reset();
    m_state = CommanderState::UNCONFIGURED;
}

void 
PadflieCommander::m_on_charged_callback() 
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}

void PadflieCommander::m_on_state_callback()
{
    if (m_hardware_actor && m_hw_state_controller->yaw_valid()) {
        m_hardware_actor->set_current_yaw(m_hw_state_controller->get_yaw());
    }
    if (!m_availability_pub ||
        !m_hw_state_controller->is_charged() ||
        !m_hw_state_controller->canfly() ||
        m_hw_state_controller->is_flying() ||
        m_hw_state_controller->is_tumbled())
    {
        return;
    }

    Eigen::Vector3d position;
    if (!m_padflie_tf->get_cf_position(position)) return;

    const auto current_site = m_site_selector->get_current_site();
    if (!current_site) {
        RCLCPP_DEBUG(m_logger, "Cannot publish availability without current SiteInfo.");
        return;
    }

    padflies_interfaces::msg::AvailabilityInfo message;
    message.name = m_prefix;
    message.site_info = *current_site;
    message.capabilities = get_hardware_capabilities();
    m_availability_pub->publish(message);
}

void PadflieCommander::m_create_availability_interface()
{
    auto options = rclcpp::PublisherOptions();
    options.callback_group = m_callback_group;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                .best_effort()
                .durability_volatile();
    m_availability_pub = rclcpp::create_publisher<padflies_interfaces::msg::AvailabilityInfo>(
        m_node_interfaces.topics_interface,
        "availability", qos, options);
}

void PadflieCommander::m_remove_availability_interface()
{
    m_availability_pub.reset();
}

void PadflieCommander::m_create_goal_interfaces()
{
    m_accepting_goals = true;
    m_deploy_action_server = rclcpp_action::create_server<padflies_interfaces::action::Deploy>(
        m_node_interfaces.base_interface,
        m_node_interfaces.clock_interface,
        m_node_interfaces.logging_interface,
        m_node_interfaces.waitables_interface,
        m_prefix + "/deploy",
        std::bind(
            &PadflieCommander::m_handle_deploy_action_goal, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(
            &PadflieCommander::m_handle_deploy_action_cancel, this,
            std::placeholders::_1),
        std::bind(
            &PadflieCommander::m_handle_deploy_action_accepted, this,
            std::placeholders::_1),
        rcl_action_server_get_default_options(),
        m_callback_group);

    m_return_action_server = rclcpp_action::create_server<padflies_interfaces::action::Return>(
        m_node_interfaces.base_interface,
        m_node_interfaces.clock_interface,
        m_node_interfaces.logging_interface,
        m_node_interfaces.waitables_interface,
        m_prefix + "/return",
        std::bind(
            &PadflieCommander::m_handle_return_action_goal, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(
            &PadflieCommander::m_handle_return_action_cancel, this,
            std::placeholders::_1),
        std::bind(
            &PadflieCommander::m_handle_return_action_accepted, this,
            std::placeholders::_1),
        rcl_action_server_get_default_options(),
        m_callback_group);

    m_cancel_completion_timer = rclcpp::create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            m_goal_manager->complete_deferred_cancellations();
            std::vector<std::shared_ptr<padflies_cpp::commander::IActionGoalCompletion>>
                completions;
            {
                std::lock_guard<std::mutex> lock(m_action_goals_mutex);
                for (const auto & [goal_id, goal] : m_action_goals) {
                    (void)goal_id;
                    completions.push_back(goal.completion);
                }
            }
            for (const auto & completion : completions) {
                completion->finish_deferred_cancellation();
            }
            m_cancel_completion_timer->cancel();
        },
        m_callback_group,
        m_node_interfaces.base_interface.get(),
        m_node_interfaces.timers_interface.get());
    m_cancel_completion_timer->cancel();
}

void PadflieCommander::m_remove_goal_interfaces()
{
    m_accepting_goals = false;
    m_cancel_completion_timer.reset();
    m_deploy_action_server.reset();
    m_return_action_server.reset();
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
    } else if (m_hw_state_controller->is_flying()) {
        m_state = CommanderState::FLYING;
    } else {
        m_state = m_hw_state_controller->is_charged() ?
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
        std::make_shared<TriggerGoalCompletion>(service_handle, request_id),
        GoalRetryPolicy::INFINITE);
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
        std::make_shared<TriggerGoalCompletion>(service_handle, request_id),
        GoalRetryPolicy::INFINITE);
}

rclcpp_action::GoalResponse PadflieCommander::m_handle_deploy_action_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const padflies_interfaces::action::Deploy::Goal>)
{
    return m_accepting_goals ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse PadflieCommander::m_handle_deploy_action_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Deploy>>
        goal_handle)
{
    return m_cancel_action_goal(goal_handle);
}

void PadflieCommander::m_handle_deploy_action_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Deploy>>
        goal_handle)
{
    using namespace padflies_cpp::commander;
    const auto goal_id = goal_handle->get_goal_id();
    auto completion = std::make_shared<ActionGoalCompletion<padflies_interfaces::action::Deploy>>(
        goal_handle,
        [this, goal_id]() {m_remove_action_goal(goal_id);});

    const auto goal = goal_handle->get_goal();
    const auto internal_id = goal->has_target ?
        m_goal_manager->request_goal(
            DeployTo{goal->target}, completion, GoalRetryPolicy::INFINITE) :
        m_goal_manager->request_goal(
            Deploy{}, completion, GoalRetryPolicy::INFINITE);

    if (goal_handle->is_active()) {
        std::lock_guard<std::mutex> lock(m_action_goals_mutex);
        m_action_goals.insert_or_assign(goal_id, ActionGoalRecord{internal_id, completion});
    }
}

rclcpp_action::GoalResponse PadflieCommander::m_handle_return_action_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const padflies_interfaces::action::Return::Goal>)
{
    return m_accepting_goals ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse PadflieCommander::m_handle_return_action_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Return>>
        goal_handle)
{
    return m_cancel_action_goal(goal_handle);
}

void PadflieCommander::m_handle_return_action_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Return>>
        goal_handle)
{
    using namespace padflies_cpp::commander;
    const auto goal_id = goal_handle->get_goal_id();
    auto completion = std::make_shared<ActionGoalCompletion<padflies_interfaces::action::Return>>(
        goal_handle,
        [this, goal_id]() {m_remove_action_goal(goal_id);});

    const auto goal = goal_handle->get_goal();
    const auto internal_id = goal->has_site ?
        m_goal_manager->request_goal(
            ReturnTo{goal->site}, completion, GoalRetryPolicy::INFINITE) :
        m_goal_manager->request_goal(
            Return{}, completion, GoalRetryPolicy::INFINITE);

    if (goal_handle->is_active()) {
        std::lock_guard<std::mutex> lock(m_action_goals_mutex);
        m_action_goals.insert_or_assign(goal_id, ActionGoalRecord{internal_id, completion});
    }
}

template<typename ActionT>
rclcpp_action::CancelResponse PadflieCommander::m_cancel_action_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle)
{
    std::shared_ptr<padflies_cpp::commander::IActionGoalCompletion> completion;
    std::uint64_t internal_id;
    {
        std::lock_guard<std::mutex> lock(m_action_goals_mutex);
        const auto goal = m_action_goals.find(goal_handle->get_goal_id());
        if (goal == m_action_goals.end()) return rclcpp_action::CancelResponse::REJECT;
        internal_id = goal->second.internal_id;
        completion = goal->second.completion;
    }

    completion->set_cancel_requested(true);
    if (!m_goal_manager->cancel_goal(internal_id)) {
        completion->set_cancel_requested(false);
        return rclcpp_action::CancelResponse::REJECT;
    }

    // A queued goal completes on the next callback, after rclcpp_action has
    // transitioned the accepted cancellation into the CANCELING state.
    m_cancel_completion_timer->reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PadflieCommander::m_remove_action_goal(const rclcpp_action::GoalUUID & goal_id)
{
    std::lock_guard<std::mutex> lock(m_action_goals_mutex);
    m_action_goals.erase(goal_id);
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
