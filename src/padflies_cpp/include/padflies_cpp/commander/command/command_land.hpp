#pragma once


#include "padflies_cpp/commander/command/command.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"

class LandCommand : public Command
{
    struct SelectionContext;

public:
    LandCommand(
      std::shared_ptr<RoutineFactory> routine_factory,
      std::shared_ptr<SiteSelector> site_selector,
      std::shared_ptr<ICompletionHandler> completion_handler = nullptr)
    : LandCommand(
        std::make_shared<SelectionContext>(
          SelectionContext{std::move(routine_factory), std::move(site_selector), ""}),
        std::move(completion_handler)) {};

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
    void succeeded() override
    {
        m_selection_context->site_selector->set_current_site(m_selection_context->selected_site);
        Command::succeeded();
    }

private:
    struct SelectionContext
    {
      std::shared_ptr<RoutineFactory> routine_factory;
      std::shared_ptr<SiteSelector> site_selector;
      std::string selected_site;
    };

    LandCommand(
      std::shared_ptr<SelectionContext> selection_context,
      std::shared_ptr<ICompletionHandler> completion_handler)
    : Command(
        [selection_context]() {
          const auto site_info = selection_context->site_selector->select_landing_site();
          if (!site_info || site_info->landing_plugin_name.empty()) {
            return std::shared_ptr<Routine>{};
          }
          selection_context->selected_site = site_info->name;
          return selection_context->routine_factory->create_land_routine(*site_info);
        },
        std::move(completion_handler)),
      m_selection_context(std::move(selection_context)) {};

    std::shared_ptr<SelectionContext> m_selection_context;
};
