#pragma once


#include "padflies_cpp/commander/command/command.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"

class LandCommand : public Command
{
public:
    LandCommand(
      std::shared_ptr<RoutineFactory> routine_factory,
      std::shared_ptr<SiteSelector> site_selector,
      rclcpp::Logger logger,
      std::shared_ptr<ICompletionHandler> completion_handler = nullptr,
      std::size_t max_retries = 50)
    : Command(logger.get_child("LandCommand"), max_retries, std::move(completion_handler)),
      m_routine_factory(std::move(routine_factory)),
      m_site_selector(std::move(site_selector)) {};

    bool 
    preconditions_are_met(const ICommandContext& context) const override
    {
        bool can_land = context.can_land();
        bool is_healthy = context.is_healthy();
        return can_land && is_healthy;
    }

    CommanderState get_target_state() const override 
    {
        if (true)    
           return CommanderState::CHARGING;
    }

    CommanderState get_working_state() const override
    {
        return CommanderState::LANDING;
    }

protected:
    bool prepare() override
    {
        const auto site_info = m_site_selector->select_landing_site(m_excluded_sites);
        if (!site_info || site_info->landing_plugin_name.empty()) {
            return false;
        }

        m_selected_site = site_info->name;
        m_routine = m_routine_factory->create_land_routine(*site_info);
        return static_cast<bool>(m_routine);
    }

    bool prepare_retry(const RoutineResult & result) override
    {
        if (result.reason == RoutineFailureReason::SITE && !m_selected_site.empty()) {
            m_excluded_sites.insert(m_selected_site);
        }
        return prepare();
    }

    void succeeded() override
    {
        m_site_selector->set_current_site(m_selected_site);
        Command::succeeded();
    }

private:
    std::shared_ptr<RoutineFactory> m_routine_factory;
    std::shared_ptr<SiteSelector> m_site_selector;
    std::string m_selected_site;
    std::set<std::string> m_excluded_sites;
};
