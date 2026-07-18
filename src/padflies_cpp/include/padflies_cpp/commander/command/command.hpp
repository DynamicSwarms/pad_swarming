#pragma once

#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/command_context_interface.hpp"
#include "padflies_cpp/commander/commander_state.hpp"

#include <mutex>
#include <condition_variable>
#include <exception>
#include <functional>

class ICompletionHandler
{
public:
  virtual void on_succeeded() {};
  virtual void on_failed() {};
  virtual void on_aborted() {};
};

class Command
{
public:
  using RoutineFactoryFunction = std::function<std::shared_ptr<Routine>()>;

  Command(
    RoutineFactoryFunction create_routine,
    std::shared_ptr<ICompletionHandler> completion_handler = nullptr)
    : m_create_routine(std::move(create_routine))
    , m_completion_handler(completion_handler)
  {}

  virtual ~Command() = default;

  void wait_until_finished() 
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this](){  return m_done; });
  }

  bool start() {
    if (m_has_been_started) {
      return false;
    }

    m_has_been_started = true;
    try {
      m_routine = m_create_routine();
    } catch (const std::exception &) {
      finish(false);
      return false;
    }
    if (!m_routine) {
      finish(false);
      return false;
    }
    m_routine->set_on_finished_callback([this](bool success)
    {
      finish(success);
    });
    m_routine->start();
    return true;
  }

  void halt() {
    if (m_routine) {
      m_routine->halt();
    }
  }

  bool has_been_started() const {
    return m_has_been_started;
  }

  bool is_running() const {
    return m_routine && m_routine->is_running();
  }

  bool is_finished () const {
    return m_done;
  }

  virtual bool preconditions_are_met(const ICommandContext& context) const = 0;

  void abort() {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_done = true;
    }

    if (m_completion_handler) {
      m_completion_handler->on_aborted();
    }

    m_cv.notify_all();
  };

  virtual CommanderState get_target_state() const = 0;
  virtual CommanderState get_working_state() const = 0;

private:

  void finish(bool success) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_done = true;
    }

    if (success) {
      succeeded();
    } else {
      failed();
    }

    m_cv.notify_all();
  }

protected:

  virtual void failed() {
    if (m_completion_handler) {
        m_completion_handler->on_failed();
    }
  };
  virtual void succeeded() {
    if (m_completion_handler) {
        m_completion_handler->on_succeeded();
    }
  };

  RoutineFactoryFunction m_create_routine;
  std::shared_ptr<Routine> m_routine;
  std::shared_ptr<ICompletionHandler> m_completion_handler;  


  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_has_been_started = false;
  bool m_done = false;
};
