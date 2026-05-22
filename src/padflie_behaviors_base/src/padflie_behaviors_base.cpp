#include "padflie_behaviors_base/padflie_behaviors_base.hpp"
#include "padflies_cpp/behavior_plugins/choose_pad.hpp"

namespace padflie_behaviors_base
{

  class ChoosePadDefault : public padflies_cpp::behavior_plugins::ChoosePad
  {
  public:
    ChoosePadDefault(
      const std::string& name,
      const BT::NodeConfig& config,
      rclcpp::Logger logger,
      std::shared_ptr<PadClientFactory> pad_client_factory)
      : padflies_cpp::behavior_plugins::ChoosePad(name, config, logger, pad_client_factory)
      , m_logger(logger.get_child(name))
      , m_pad_client_factory(pad_client_factory)
    {
    }

    BT::NodeStatus tick() override
    {
      RCLCPP_INFO(m_logger, "Choosing a pad and creating PadClient...");
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client("megapad");
  
      setOutput("pad_client", pad_client);
      return BT::NodeStatus::SUCCESS;
    }
    private:
      rclcpp::Logger m_logger;
      std::shared_ptr<PadClientFactory> m_pad_client_factory;

  };
  
}  // namespace padflie_behaviors_base


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(padflie_behaviors_base::ChoosePadDefault, padflies_cpp::behavior_plugins::ChoosePad)