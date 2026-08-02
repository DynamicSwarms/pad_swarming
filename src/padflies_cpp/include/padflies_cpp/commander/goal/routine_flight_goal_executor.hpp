#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"
#include "padflies_cpp/commander/goal/flight_goal_executor.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"

namespace padflies_cpp::commander
{

class RoutineFlightGoalExecutor final : public IFlightGoalExecutor
{
public:
  using GoalFinished = std::function<void(FlightGoalKind, GoalResult)>;
  using GoalStarted = std::function<void(FlightGoalKind)>;
  using HardwareIsFlying = std::function<bool()>;

  RoutineFlightGoalExecutor(
    std::shared_ptr<RoutineFactory> routine_factory,
    std::shared_ptr<SiteSelector> site_selector,
    HardwareIsFlying hardware_is_flying,
    GoalStarted goal_started,
    GoalFinished goal_finished)
  : m_routine_factory(std::move(routine_factory)),
    m_site_selector(std::move(site_selector)),
    m_hardware_is_flying(std::move(hardware_is_flying)),
    m_goal_started(std::move(goal_started)),
    m_goal_finished(std::move(goal_finished))
  {
    if (!m_routine_factory || !m_site_selector || !m_hardware_is_flying ||
      !m_goal_started || !m_goal_finished)
    {
      throw std::invalid_argument("RoutineFlightGoalExecutor dependencies must not be null");
    }
  }

  void start_routine_for_goal(
    std::uint64_t,
    const FlightGoal & goal,
    GoalRetryPolicy retry_policy) override
  {
    if (m_active_routine) {
      throw std::logic_error("A flight goal routine is already active");
    }

    m_active_goal = goal;
    m_active_goal_kind = kind(m_active_goal);
    m_retry_policy = retry_policy;
    if (goal_is_already_satisfied()) {
      report_goal_finished({GoalOutcome::SUCCESS, "Hardware is already in the requested state"});
      return;
    }
    m_goal_started(m_active_goal_kind);
    start_routine_attempt();
  }

  bool interrupt_active_routine_if_possible() override
  {
    return m_active_routine && m_active_routine->request_interruption_if_possible();
  }

  void stop_active_routine_without_callback() override
  {
    if (!m_active_routine) return;

    m_active_routine->set_on_finished_callback({});
    m_active_routine->halt();
    m_active_routine.reset();
  }

private:
  void start_routine_attempt()
  {
    try {
      m_active_routine = create_routine(m_active_goal);
    } catch (const std::exception & exception) {
      report_goal_finished({GoalOutcome::FAILURE, exception.what()});
      return;
    }
    if (!m_active_routine) {
      report_goal_finished({GoalOutcome::FAILURE, "No suitable flight goal routine available"});
      return;
    }

    m_active_routine->set_on_finished_callback(
      [this](RoutineResult result) {routine_finished(std::move(result));});
    m_active_routine->start();
  }
  bool goal_is_already_satisfied() const
  {
    const bool deploy =
      m_active_goal_kind == FlightGoalKind::DEPLOY ||
      m_active_goal_kind == FlightGoalKind::DEPLOY_TO;
    return deploy == m_hardware_is_flying();
  }

  std::shared_ptr<Routine> create_routine(const FlightGoal & goal)
  {
    return std::visit(
      [this](const auto & requested_goal) -> std::shared_ptr<Routine> {
        using Goal = std::decay_t<decltype(requested_goal)>;
        if constexpr (std::is_same_v<Goal, Deploy> || std::is_same_v<Goal, DeployTo>) {
          return create_deploy_routine();
        } else if constexpr (std::is_same_v<Goal, Return>) {
          return create_return_routine();
        } else {
          return create_return_routine_to(requested_goal.site);
        }
      }, goal);
  }

  std::shared_ptr<Routine> create_deploy_routine()
  {
    const auto site = m_site_selector->select_takeoff_site();
    if (!site || site->takeoff_plugin_name.empty()) return {};
    m_site_after_success.clear();
    return m_routine_factory->create_takeoff_routine(*site);
  }

  std::shared_ptr<Routine> create_return_routine()
  {
    const auto site = m_site_selector->select_landing_site();
    if (!site) return {};
    m_site_after_success = site->name;
    return m_routine_factory->create_land_routine(*site);
  }

  std::shared_ptr<Routine> create_return_routine_to(const std::string & site_name)
  {
    const auto site = m_site_selector->select_landing_site(site_name);
    if (!site) return {};
    m_site_after_success = site->name;
    return m_routine_factory->create_land_routine(*site);
  }

  void routine_finished(RoutineResult result)
  {
    // Keep the routine alive until this callback and any synchronous goal
    // handover have completed.
    auto finished_routine = std::move(m_active_routine);
    (void)finished_routine;

    if (result.outcome == RoutineOutcome::SUCCESS) {
      m_site_selector->set_current_site(m_site_after_success);
      report_goal_finished({GoalOutcome::SUCCESS, std::move(result.message)});
    } else if (result.outcome == RoutineOutcome::INTERRUPTED) {
      report_goal_finished({GoalOutcome::INTERRUPTED, std::move(result.message)});
    } else if (
      result.outcome == RoutineOutcome::RETRY &&
      m_retry_policy == GoalRetryPolicy::INFINITE)
    {
      start_routine_attempt();
    } else {
      report_goal_finished({GoalOutcome::FAILURE, std::move(result.message)});
    }
  }

  void report_goal_finished(GoalResult result)
  {
    m_goal_finished(m_active_goal_kind, std::move(result));
  }

  std::shared_ptr<RoutineFactory> m_routine_factory;
  std::shared_ptr<SiteSelector> m_site_selector;
  HardwareIsFlying m_hardware_is_flying;
  GoalStarted m_goal_started;
  GoalFinished m_goal_finished;
  std::shared_ptr<Routine> m_active_routine;
  FlightGoal m_active_goal{Deploy{}};
  std::string m_site_after_success;
  FlightGoalKind m_active_goal_kind{FlightGoalKind::DEPLOY};
  GoalRetryPolicy m_retry_policy{GoalRetryPolicy::NEVER};
};

}  // namespace padflies_cpp::commander
