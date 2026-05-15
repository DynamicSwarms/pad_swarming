#include "rqt_padflies/manager_plugin.hpp"

#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>

namespace rqt_padflies
{


void ManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  m_node = node_;
  m_widget = new QWidget();
  m_ui.setupUi(m_widget);


  connect(this, &ManagerPlugin::add_padflie, this, &ManagerPlugin::m_signal_handler_add_padflie);
  connect(this, &ManagerPlugin::availability_message_received, this, &ManagerPlugin::m_signal_handler_availability_message);

  m_widget->setWindowTitle("Padflies Manager");
  context.addWidget(m_widget);

  m_update_timer = rclcpp::create_timer(
    m_node->get_node_base_interface(),
    m_node->get_node_timers_interface(),
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), // Dont run faster if use_sim_time is enabled
    std::chrono::milliseconds(500),
    std::bind(&ManagerPlugin::update, this)
  ); 

  m_availability_subscription = m_node->create_subscription<std_msgs::msg::String>(
    "availability", 10, 
    std::bind(&ManagerPlugin::m_handle_availability_message, this, std::placeholders::_1)
  );

}

void
ManagerPlugin::update()
{
  std::vector<std::string> node_names = m_node->get_node_graph_interface()->get_node_names();
  for (const auto& node_name : node_names) {
    if (node_name.find("padflie") != std::string::npos) {
      try {
        std::string id_str = node_name.substr(8); // Assuming name is like "/padflieID"
        int id = std::stoi(id_str);
        if (m_padflie_widgets.find(id) == m_padflie_widgets.end()) {
          emit add_padflie(id);
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(m_node->get_logger(), "Found a node with name containing 'padflie' but failed to extract ID: %s", node_name.c_str());
      }
    }
  }
};

void ManagerPlugin::m_handle_availability_message(std::shared_ptr<std_msgs::msg::String> msg)
{
  try {
    std::string id_str = msg->data.substr(8); // Assuming name is like "/padflieID"
    int id = std::stoi(id_str);
    emit availability_message_received(id);
  } catch (const std::exception& e) {
    RCLCPP_WARN(m_node->get_logger(), "Received availability message but failed to extract ID: %s", msg->data.c_str());
  }
}


void
ManagerPlugin::m_signal_handler_add_padflie(int id)
{
  if (m_padflie_widgets.find(id) != m_padflie_widgets.end()) {
    return;
  }

  RCLCPP_INFO(m_node->get_logger(), "Adding padflie with ID %d to the manager", id);
  m_padflie_widgets[id] = new PadflieListWidgetItem(id);
  PadflieWidget * widget = m_padflie_widgets[id]->get_widget();
  m_padflie_widgets[id]->setSizeHint(QSize(500, widget->getHeight()));


  m_ui.list_widget->addItem(m_padflie_widgets[id]);
  m_ui.list_widget->setItemWidget(m_padflie_widgets[id], widget);
  m_ui.list_widget->sortItems();
}

void 
ManagerPlugin::m_signal_handler_availability_message(int id)
{
  //RCLCPP_INFO(m_node->get_logger(), "Received availability message for padflie with ID %d", id);
  if (m_padflie_widgets.find(id) != m_padflie_widgets.end()) {
    m_padflie_widgets[id]->set_available();
  } else {
    emit add_padflie(id);
  }
}

void ManagerPlugin::shutdownPlugin()
{
  m_update_timer->cancel();
}

void ManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{}

void ManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{}

}  // namespace rqt_padflies

PLUGINLIB_EXPORT_CLASS(rqt_padflies::ManagerPlugin, rqt_gui_cpp::Plugin)