#pragma once


#include "padflies_cpp/commander/command/command.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"

class TakeoffCommand : public Command
{
public:
    TakeoffCommand(
        std::shared_ptr<RoutineFactory> routine_factory,
        std::shared_ptr<SiteSelector> site_selector,
        rclcpp::Logger logger,
        std::shared_ptr<ICompletionHandler> completion_handler = nullptr,
        std::size_t max_retries = 0)
      : Command(logger.get_child("TakeoffCommand"), max_retries, std::move(completion_handler)),
        m_routine_factory(std::move(routine_factory)),
        m_site_selector(std::move(site_selector)) {};

    bool preconditions_are_met(const ICommandContext& context) const override
    {
        return context.is_healthy();
    }

    CommanderState get_target_state() const override 
    {
      if (true)    
           return CommanderState::FLYING;
    }

    CommanderState get_working_state() const override
    {
        return CommanderState::TAKEOFF;
    }

protected:
    bool prepare() override
    {
        const auto site_info = m_site_selector->select_takeoff_site();
        if (!site_info || site_info->takeoff_plugin_name.empty()) {
            return false;
        }

        m_routine = m_routine_factory->create_takeoff_routine(*site_info);
        return static_cast<bool>(m_routine);
    }

    void succeeded() override
    {
        m_site_selector->set_current_site("");
        Command::succeeded();
    }

private:
    std::shared_ptr<RoutineFactory> m_routine_factory;
    std::shared_ptr<SiteSelector> m_site_selector;
};
