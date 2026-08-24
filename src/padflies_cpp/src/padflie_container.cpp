#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rcpputils/split.hpp"

class PadflieContainerException : public std::runtime_error
{
public:
  explicit PadflieContainerException(const std::string & error_description)
  : std::runtime_error(error_description) {}
};

struct DedicatedExecutorWrapper
{
  std::shared_ptr<rclcpp::Executor> executor;
  std::thread thread;
  std::atomic_bool thread_initialized;

  explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> executor_in)
  : executor(std::move(executor_in)), thread_initialized(false) {}
};

class PadflieContainer : public rclcpp::Node
{
public:
  explicit PadflieContainer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("padflie_container", options),
    m_padflie_ids(
      declare_parameter<std::vector<int64_t>>("padflie_ids", std::vector<int64_t>{})),
    m_initial_site(declare_parameter<std::string>("initial_site", "megapad")),
    m_battery_voltage_charged(declare_parameter<double>("battery_voltage_charged", 4.1)),
    m_sensor_logging_profiles(
      declare_parameter<std::string>("sensor_logging_profiles", ""))
  {
    m_factory = create_component_factory("padflies_cpp", "Padflie");
    if (!m_factory) {
      throw PadflieContainerException("Could not create the Padflie component factory");
    }

    m_initialize_timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PadflieContainer::initialize_padflies, this));
    RCLCPP_INFO(get_logger(), "Padflie container started.");
  }

  ~PadflieContainer() override
  {
    while (!m_padflies.empty()) {
      remove_padflie(m_padflies.begin()->first);
    }
    RCLCPP_INFO(get_logger(), "Removed all Padflies.");
  }

private:
  using PadflieEntry =
    std::pair<rclcpp_components::NodeInstanceWrapper, DedicatedExecutorWrapper>;

  void initialize_padflies()
  {
    for (const auto id : m_padflie_ids) {
      if (id < 0 || id > UINT8_MAX) {
        throw PadflieContainerException(
                "Padflie id is outside the uint8 range: " + std::to_string(id));
      }
      add_padflie(static_cast<uint8_t>(id));
    }

    RCLCPP_INFO(get_logger(), "Initialized %zu Padflies.", m_padflies.size());
    m_initialize_timer->cancel();
  }

  bool add_padflie(uint8_t id)
  {
    if (m_padflies.count(id) != 0U) {
      RCLCPP_INFO(get_logger(), "Padflie with id '%u' already exists.", id);
      return false;
    }

    try {
      auto node = m_factory->create_node_instance(create_node_options(id));
      auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor->add_node(node.get_node_base_interface());

      auto entry = m_padflies.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(std::move(node), executor));
      auto & wrapper = entry.first->second.second;
      auto & thread_initialized = wrapper.thread_initialized;
      wrapper.thread = std::thread(
        [executor, &thread_initialized]() {
          thread_initialized = true;
          try {
            executor->spin();
          } catch (const std::exception &) {
            // The owning container handles shutdown and removes the node.
          }
        });
    } catch (const std::exception & exception) {
      throw PadflieContainerException(
              "Failed to create Padflie " + std::to_string(id) + ": " + exception.what());
    } catch (...) {
      throw PadflieContainerException(
              "Padflie " + std::to_string(id) + " constructor threw an exception");
    }

    return true;
  }

  bool remove_padflie(uint8_t id)
  {
    auto padflie = m_padflies.find(id);
    if (padflie == m_padflies.end()) {
      RCLCPP_INFO(get_logger(), "Padflie with id '%u' does not exist.", id);
      return false;
    }

    while (!padflie->second.second.thread_initialized) {
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    padflie->second.second.executor->cancel();
    padflie->second.second.thread.join();
    m_padflies.erase(padflie);
    return true;
  }

  rclcpp::NodeOptions create_node_options(uint8_t id) const
  {
    std::vector<std::string> arguments{
      "--ros-args", "-r", "__node:=padflie" + std::to_string(id),
      "-p", "id:=" + std::to_string(id),
      "-p", "initial_site:=" + m_initial_site,
      "-p", "battery_voltage_charged:=" + std::to_string(m_battery_voltage_charged),
      "-p", "sensor_logging_profiles:=" + m_sensor_logging_profiles};
    return rclcpp::NodeOptions().arguments(arguments);
  }

  std::vector<std::pair<std::string, std::string>> get_component_resources(
    const std::string & package_name, const std::string & resource_index) const
  {
    const auto resource = ament_index_cpp::get_resource(resource_index, package_name);
    if (!resource.resourcePath) {
      throw PadflieContainerException("Could not find requested resource in ament index");
    }

    std::vector<std::pair<std::string, std::string>> resources;
    for (const auto & line : rcpputils::split(resource.contents, '\n', true)) {
      const auto parts = rcpputils::split(line, ';');
      if (parts.size() != 2U) {
        throw PadflieContainerException("Invalid component resource entry");
      }
      std::filesystem::path library_path = parts[1];
      if (!library_path.is_absolute()) {
        library_path = *resource.resourcePath / library_path;
      }
      resources.emplace_back(parts[0], library_path.string());
    }
    return resources;
  }

  std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(
    const std::string & package_name, const std::string & class_name)
  {
    const auto resources = get_component_resources(package_name, "rclcpp_components");
    if (resources.empty()) {
      throw PadflieContainerException("No Padflie component resource was registered");
    }

    const auto resource = std::find_if(
      resources.begin(), resources.end(),
      [&class_name](const auto & item) {return item.first == class_name;});
    if (resource == resources.end()) {
      throw PadflieContainerException("The Padflie component resource was not registered");
    }

    RCLCPP_INFO(get_logger(), "Load library: %s", resource->second.c_str());
    m_loader = std::make_unique<class_loader::ClassLoader>(resource->second);
    const std::string factory_class =
      "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

    for (const auto & available_class :
      m_loader->getAvailableClasses<rclcpp_components::NodeFactory>())
    {
      RCLCPP_INFO(get_logger(), "Found class: %s", available_class.c_str());
      if (available_class == class_name || available_class == factory_class) {
        return m_loader->createInstance<rclcpp_components::NodeFactory>(available_class);
      }
    }
    return {};
  }

  rclcpp::TimerBase::SharedPtr m_initialize_timer;
  std::vector<int64_t> m_padflie_ids;
  std::string m_initial_site;
  double m_battery_voltage_charged;
  std::string m_sensor_logging_profiles;
  std::unique_ptr<class_loader::ClassLoader> m_loader;
  std::shared_ptr<rclcpp_components::NodeFactory> m_factory;
  std::map<uint8_t, PadflieEntry> m_padflies;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto container = std::make_shared<PadflieContainer>();
  executor->add_node(container);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
