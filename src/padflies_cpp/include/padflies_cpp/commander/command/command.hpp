#pragma once

#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/command_context_interface.hpp"
#include "padflies_cpp/commander/commander_state.hpp"

#include <mutex>
#include <condition_variable>
#include <exception>

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
  explicit Command(
    rclcpp::Logger logger,
    std::size_t max_retries,
    std::shared_ptr<ICompletionHandler> completion_handler = nullptr)
    : m_logger(std::move(logger))
    , m_completion_handler(std::move(completion_handler))
    , m_max_retries(max_retries)
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
    return prepare_and_start(false, {});
  }

  void update()
  {
    RoutineResult retry_result;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_retry_pending || m_done) {
        return;
      }
      retry_result = m_retry_result;
      m_retry_pending = false;
    }

    m_routine.reset();
    prepare_and_start(true, retry_result);
  }

  void halt() {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_halt_requested = true;
    }
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

  bool prepare_and_start(bool retry, const RoutineResult & retry_result)
  {
    try {
      const bool prepared = retry ? prepare_retry(retry_result) : prepare();
      if (!prepared || !m_routine) {
        finish({RoutineOutcome::FAILURE, retry_result.reason,
          retry ? "Could not prepare routine retry" : "Could not prepare routine"});
        return false;
      }
    } catch (const std::exception & exception) {
      finish({RoutineOutcome::FAILURE, retry_result.reason, exception.what()});
      return false;
    }

    m_routine->set_on_finished_callback([this](RoutineResult result)
    {
      on_routine_finished(std::move(result));
    });
    m_routine->start();
    return true;
  }

  void on_routine_finished(RoutineResult result)
  {
    if (result.outcome == RoutineOutcome::RETRY) {
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_halt_requested) {
          result.outcome = RoutineOutcome::FAILURE;
        } else if (m_retry_count >= m_max_retries) {
          result.outcome = RoutineOutcome::FAILURE;
          result.message = "Maximum routine retries reached";
        } else {
          ++m_retry_count;
          m_retry_result = result;
          m_retry_pending = true;
        }
      }

      if (result.outcome == RoutineOutcome::RETRY) {
        RCLCPP_WARN(
          m_logger, "Routine failed; scheduling retry %zu/%zu: %s",
          m_retry_count, m_max_retries, result.message.c_str());
        return;
      }
    }

    if (result.outcome == RoutineOutcome::SUCCESS) {
      RCLCPP_INFO(m_logger, "Routine succeeded after %zu retries.", m_retry_count);
    } else {
      RCLCPP_ERROR(
        m_logger, "Routine failed after %zu retries: %s",
        m_retry_count, result.message.c_str());
    }
    finish(std::move(result));
  }

  void finish(RoutineResult result) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_done = true;
    }

    if (result.outcome == RoutineOutcome::SUCCESS) {
      succeeded();
    } else {
      failed();
    }

    m_cv.notify_all();
  }

protected:

  virtual bool prepare() = 0;
  virtual bool prepare_retry(const RoutineResult &)
  {
    return prepare();
  }

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

  rclcpp::Logger m_logger;
  std::shared_ptr<Routine> m_routine;
  std::shared_ptr<ICompletionHandler> m_completion_handler;  


  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_has_been_started = false;
  bool m_done = false;
  bool m_halt_requested = false;
  bool m_retry_pending = false;
  const std::size_t m_max_retries;
  std::size_t m_retry_count = 0;
  RoutineResult m_retry_result;
};
