
#include "padflies_cpp/routine_factory.hpp"

#include "padflies_cpp/behaviors.hpp"


RoutineFactory::RoutineFactory(
    padflies_cpp::NodeInterfacesBundle node_interfaces_bundle,
    rclcpp::Logger logger)
  : m_behavior_plugin_loader("padflies_cpp", "padflies_cpp::IPadflieBehaviorPlugin")
  , m_param_callback_handle(node_interfaces_bundle.parameters_interface->add_on_set_parameters_callback(std::bind(&RoutineFactory::m_set_parameters_callback, this, std::placeholders::_1)))
  , m_node_interfaces_bundle(node_interfaces_bundle)
  , m_logger(logger)
  {
      m_change_plugin_timer = rclcpp::create_timer(
        m_node_interfaces_bundle.base_interface,
        m_node_interfaces_bundle.timers_interface,
        m_node_interfaces_bundle.clock_interface->get_clock(),
        std::chrono::milliseconds(0), // One-shot timer to trigger immediately
        std::bind(&RoutineFactory::m_set_plugin, this)
    );


    // Declaring will automatically load the default plugin.
    m_node_interfaces_bundle.parameters_interface->declare_parameter("behavior_plugin_name", rclcpp::ParameterValue("simpleflie_behaviors::SimpleflieBehaviors")); 
  }

// RoutineFactory::~RoutineFactory()
// {
//   RCLCPP_INFO(m_logger, "RoutineFactory destructor called.");
// }


void
RoutineFactory::set_padflie_shared_ptrs(
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory
)
{
  if (!hardware_actor || !padflie_tf || !pad_execute_server || !pad_client_factory) {
    RCLCPP_ERROR(m_logger, "One or more shared pointers are null in set_padflie_shared_ptrs");
    throw std::invalid_argument("Null shared pointer provided to set_padflie_shared_ptrs");
  }

  m_hardware_actor = hardware_actor;
  m_padflie_tf = padflie_tf;
  m_pad_execute_server = pad_execute_server;
  m_pad_client_factory = pad_client_factory;
}

void 
RoutineFactory::reset_padflie_shared_ptrs()
{

  m_hardware_actor.reset();
  m_padflie_tf.reset();
  m_pad_execute_server.reset();
  m_pad_client_factory.reset();
}

std::shared_ptr<Routine>
RoutineFactory::create_takeoff_routine()
{
  BT::BehaviorTreeFactory factory;
  m_register_base_nodes(factory, m_hardware_actor, m_padflie_tf, m_pad_execute_server, m_pad_client_factory);
  return std::make_shared<Routine>(
    p_plugin->getTakeoffTree(factory, m_hardware_actor, m_padflie_tf, m_pad_execute_server, m_pad_client_factory),
    m_node_interfaces_bundle.base_interface,
    m_node_interfaces_bundle.timers_interface,
    m_node_interfaces_bundle.clock_interface,
    m_logger);
}

std::shared_ptr<Routine>
RoutineFactory::create_land_routine()
{
  BT::BehaviorTreeFactory factory;
  m_register_base_nodes(factory, m_hardware_actor, m_padflie_tf, m_pad_execute_server, m_pad_client_factory);
  return std::make_shared<Routine>(
    p_plugin->getLandTree(factory, m_hardware_actor, m_padflie_tf, m_pad_execute_server, m_pad_client_factory),
    m_node_interfaces_bundle.base_interface,
    m_node_interfaces_bundle.timers_interface,
    m_node_interfaces_bundle.clock_interface,
    m_logger);
}

void
RoutineFactory::m_register_base_nodes(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory
)
{
  factory.registerNodeType<GetPadRight>("GetPadRight", m_logger, m_pad_execute_server);
  factory.registerNodeType<HoldPadRight>("HoldPadRight", m_logger, m_pad_execute_server);
  factory.registerNodeType<ReleasePadRight>("ReleasePadRight", m_logger, m_pad_execute_server);
  factory.registerNodeType<TakeoffInit>("TakeoffInit", m_logger);
  factory.registerNodeType<TakeoffRoutine>("Takeoff", m_logger, m_node_interfaces_bundle.clock_interface, m_hardware_actor, m_padflie_tf, m_pad_execute_server);
  factory.registerNodeType<LandRoutine>("Land", m_logger, m_node_interfaces_bundle.clock_interface, m_hardware_actor,m_padflie_tf, m_pad_execute_server);
  factory.registerNodeType<LandInit>("LandInit", m_logger);
  factory.registerNodeType<ApproachIDLE>("ApproachIDLE", m_logger, m_hardware_actor, m_padflie_tf, m_pad_execute_server);
  factory.registerNodeType<ApproachCLOSE>("ApproachCLOSE", m_logger, m_hardware_actor, m_padflie_tf,  m_pad_execute_server);
  factory.registerNodeType<TimeoutROS>("TimeoutROS", m_logger, m_node_interfaces_bundle.clock_interface);
  factory.registerNodeType<TryFinally>("TryFinally", m_logger);
  factory.registerNodeType<SendFeedback>("SendFeedback", m_logger, m_pad_execute_server);
  factory.registerNodeType<HasPadRight>("HasPadRight", m_logger);
}

rcl_interfaces::msg::SetParametersResult 
RoutineFactory::m_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{ 
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& param : parameters) 
  {
    if (param.get_name() == "behavior_plugin_name") {
      if (m_behavior_plugin_loader.isClassAvailable(param.as_string()))
      {
          result.successful = true;
          m_plugin_name = param.as_string(); 
          m_change_plugin_timer->reset(); 
      } else {
        result.successful = false;
        result.reason = "Plugin not found";  
        RCLCPP_ERROR(m_logger, "Plugin '%s' not found in plugin loader", param.as_string().c_str());
      }
    }
  }
  return result;
}

void 
RoutineFactory::m_set_plugin()
{
  m_change_plugin_timer->cancel(); // OneShot Timer -> cancel after trigger
  try {
    p_plugin = m_behavior_plugin_loader.createSharedInstance(m_plugin_name, m_node_interfaces_bundle, m_logger);
    RCLCPP_INFO(m_logger, "Successfully loaded behavior plugin: %s", m_plugin_name.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(m_logger, "Failed to load behavior plugin: %s. Exception: %s", m_plugin_name.c_str(), ex.what());
  }
}
    