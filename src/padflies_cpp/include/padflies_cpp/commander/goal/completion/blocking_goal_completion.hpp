#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>

#include "padflies_cpp/commander/goal/completion/goal_completion.hpp"

namespace padflies_cpp::commander
{

class BlockingGoalCompletion final : public IGoalCompletion
{
public:
  void complete(const GoalResult & result) override
  {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_result = result;
    }
    m_finished.notify_all();
  }

  GoalResult wait()
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_finished.wait(lock, [this]() {return m_result.has_value();});
    return *m_result;
  }

private:
  std::mutex m_mutex;
  std::condition_variable m_finished;
  std::optional<GoalResult> m_result;
};

}  // namespace padflies_cpp::commander
