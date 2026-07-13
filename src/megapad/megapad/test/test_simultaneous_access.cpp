#include <algorithm>
#include <array>
#include <barrier>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "megapad/megapad_resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using pad_management_cpp::AccessRequest;
using pad_management_cpp::AccessResponse;
using pad_management_cpp::ExecuteResult;
using pad_management_cpp::NodeInterfacesBundle;

class ResourceManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("megapad_resource_manager_test");
    NodeInterfacesBundle interfaces{
      node_->get_node_base_interface(),
      node_->get_node_topics_interface(),
      node_->get_node_services_interface(),
      node_->get_node_parameters_interface(),
      node_->get_node_timers_interface(),
      node_->get_node_clock_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_waitables_interface()};
    manager_ = std::make_unique<megapad::MegaPadResourceManager>(interfaces);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<megapad::MegaPadResourceManager> manager_;
};

TEST_F(ResourceManagerTest, GrantsOneOfFiveSimultaneousRequests)
{
  constexpr std::size_t request_count = 5;
  std::array<pad_management_cpp::AccessHandle, request_count> handles{};
  std::array<AccessResponse::Result, request_count> responses{};

  for (std::size_t i = 0; i < request_count; ++i) {
    AccessRequest request{};
    request.id = static_cast<uint8_t>(i);
    request.max_wait_time = std::chrono::seconds(10);
    request.usage_time = std::chrono::seconds(1);
    handles[i] = manager_->submit_access_request(request);
  }

  std::barrier start_gate(static_cast<std::ptrdiff_t>(request_count));
  std::array<std::thread, request_count> clients;
  for (std::size_t i = 0; i < request_count; ++i) {
    clients[i] = std::thread([&, i]() {
      start_gate.arrive_and_wait();
      responses[i] = manager_->query_request_status(handles[i]).result;
    });
  }
  for (auto & client : clients) {
    client.join();
  }

  std::size_t accepted_count = 0;
  std::size_t pending_count = 0;
  std::size_t holder = 0;
  for (std::size_t i = 0; i < request_count; ++i) {
    if (responses[i] == AccessResponse::Result::ACCEPTED) {
      ++accepted_count;
      holder = i;
    } else if (responses[i] == AccessResponse::Result::PENDING) {
      ++pending_count;
    }
  }

  ASSERT_EQ(accepted_count, 1U);
  ASSERT_EQ(pending_count, 4U);

  manager_->notify_finished(handles[holder], ExecuteResult{});

  std::size_t next_accepted_count = 0;
  for (std::size_t i = 0; i < request_count; ++i) {
    if (
      i != holder &&
      manager_->query_request_status(handles[i]).result ==
      AccessResponse::Result::ACCEPTED)
    {
      ++next_accepted_count;
    }
  }
  EXPECT_EQ(next_accepted_count, 1U);
}

TEST_F(ResourceManagerTest, EventuallyGrantsAccessToAllFiveRequests)
{
  constexpr std::size_t request_count = 5;
  std::array<pad_management_cpp::AccessHandle, request_count> handles{};
  std::array<AccessResponse::Result, request_count> responses{};
  std::array<bool, request_count> has_had_access{};

  for (std::size_t i = 0; i < request_count; ++i) {
    AccessRequest request{};
    request.id = static_cast<uint8_t>(i);
    request.max_wait_time = std::chrono::seconds(10);
    request.usage_time = std::chrono::seconds(1);
    handles[i] = manager_->submit_access_request(request);
  }

  std::barrier start_gate(static_cast<std::ptrdiff_t>(request_count));
  std::array<std::thread, request_count> clients;
  for (std::size_t i = 0; i < request_count; ++i) {
    clients[i] = std::thread([&, i]() {
      start_gate.arrive_and_wait();
      responses[i] = manager_->query_request_status(handles[i]).result;
    });
  }
  for (auto & client : clients) {
    client.join();
  }

  std::size_t access_count = 0;
  for (std::size_t i = 0; i < request_count; ++i) {
    if (responses[i] == AccessResponse::Result::ACCEPTED) {
      has_had_access[i] = true;
      ++access_count;
    }
  }
  ASSERT_EQ(access_count, 1U);

  while (access_count < request_count) {
    std::size_t current_holder = request_count;
    for (std::size_t i = 0; i < request_count; ++i) {
      if (has_had_access[i] && responses[i] == AccessResponse::Result::ACCEPTED) {
        current_holder = i;
        break;
      }
    }
    ASSERT_LT(current_holder, request_count);

    manager_->notify_finished(handles[current_holder], ExecuteResult{});
    responses[current_holder] = AccessResponse::Result::REJECTED;

    std::size_t newly_accepted = 0;
    for (std::size_t i = 0; i < request_count; ++i) {
      if (has_had_access[i]) {
        continue;
      }

      responses[i] = manager_->query_request_status(handles[i]).result;
      if (responses[i] == AccessResponse::Result::ACCEPTED) {
        has_had_access[i] = true;
        ++access_count;
        ++newly_accepted;
      }
    }
    ASSERT_EQ(newly_accepted, 1U);
  }

  EXPECT_TRUE(
    std::all_of(has_had_access.begin(), has_had_access.end(), [](bool value) {
      return value;
    }));
}
}  // namespace
