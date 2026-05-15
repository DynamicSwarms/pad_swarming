#pragma once

#include "padflies_cpp/routine.hpp"
#include "padflies_cpp/command_context_interface.hpp"
#include "padflies_cpp/commander_state.hpp"


class Command
{
public:
  Command(std::shared_ptr<Routine> routine)
    : m_routine(std::move(routine))
  {
    m_routine->set_on_finished_callback([this](bool success){
      if (success) {
        this->succeeded();
      } else {
        this->failed();
      }
    });
  }

  virtual ~Command() = default;

  void start() {
    m_routine->start();
  }

  bool has_been_started() const {
    return m_routine->has_been_started();
  }

  bool is_running() const {
    return m_routine->is_running();
  }

  bool is_finished () const {
    return m_routine->is_finished();
  }

  virtual bool preconditions_are_met(const ICommandContext& context) const = 0;

  virtual void abort() = 0;

  virtual CommanderState get_target_state() const = 0;
  virtual CommanderState get_working_state() const = 0;

private: 

  virtual void failed() = 0;
  virtual void succeeded() = 0;

protected: 
  std::shared_ptr<Routine> m_routine;
};