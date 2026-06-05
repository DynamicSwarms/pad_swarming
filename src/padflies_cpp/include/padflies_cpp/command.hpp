#pragma once

#include "padflies_cpp/routine.hpp"
#include "padflies_cpp/command_context_interface.hpp"
#include "padflies_cpp/commander_state.hpp"

#include <mutex>
#include <condition_variable>

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
  Command(
    std::shared_ptr<Routine> routine, 
    std::shared_ptr<ICompletionHandler> completion_handler = nullptr)
    : m_routine(std::move(routine))
    , m_completion_handler(completion_handler)
  {
    m_routine->set_on_finished_callback([this](bool success)
    {
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_done = true;
      }

      if (success) {
        this->succeeded();
      } else {
        this->failed();
      }

      m_cv.notify_all();
    });
  }

  virtual ~Command() = default;

  void wait_until_finished() 
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this](){  return m_done; });
  }

  void start() {
    m_routine->start();
  }

  void halt() {
    m_routine->halt();
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

  void failed() {
    if (m_completion_handler) {
        m_completion_handler->on_failed();
    }
  };
  void succeeded() {
    if (m_completion_handler) {
        m_completion_handler->on_succeeded();
    }
  };

protected: 
  std::shared_ptr<Routine> m_routine;
  std::shared_ptr<ICompletionHandler> m_completion_handler;  


  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_done = false;
};