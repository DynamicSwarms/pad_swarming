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
  m_ui.list_widget->setUniformItemSizes(true);


  connect(this, &ManagerPlugin::add_padflie,
          this, &ManagerPlugin::m_signal_handler_add_padflie,
          Qt::QueuedConnection);

  m_widget->setWindowTitle("Padflies Manager");
  context.addWidget(m_widget);

  m_update_timer = rclcpp::create_timer(
    m_node->get_node_base_interface(),
    m_node->get_node_timers_interface(),
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), // Dont run faster if use_sim_time is enabled
    std::chrono::milliseconds(500),
    std::bind(&ManagerPlugin::update, this)
  ); 

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                  .best_effort()
                  .durability_volatile();
  m_availability_subscription = m_node->create_subscription<padflies_interfaces::msg::AvailabilityInfo>(
    "availability", qos,
    std::bind(&ManagerPlugin::m_handle_availability_message, this, std::placeholders::_1)
  );

}

void
ManagerPlugin::update()
{
  if (!rclcpp::ok()) return;

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

void ManagerPlugin::m_handle_availability_message(
  std::shared_ptr<padflies_interfaces::msg::AvailabilityInfo> msg)
{
  try {
    std::string id_str = msg->name.substr(7); // Assuming name is like "padflieID"
    int id = std::stoi(id_str);
    QMetaObject::invokeMethod(
      this,
      [this, id]() {
        emit availability_message_received(id);
      },
      Qt::QueuedConnection);
  } catch (const std::exception& e) {
    RCLCPP_WARN(m_node->get_logger(), "Received availability message but failed to extract ID: %s", msg->name.c_str());
  }
}


void
ManagerPlugin::m_signal_handler_add_padflie(int id)
{
  if (m_padflie_widgets.find(id) != m_padflie_widgets.end()) {
    return;
  }

  RCLCPP_INFO(m_node->get_logger(), "Adding padflie with ID %d to the manager", id);
  std::shared_ptr<PadflieROSConnection> ros_connection = std::make_shared<PadflieROSConnection>(
    "/padflie" + std::to_string(id),
    m_node->get_node_timers_interface(),
    m_node->get_node_topics_interface(),
    m_node->get_node_base_interface(),
    m_node->get_node_graph_interface(),
    m_node->get_node_services_interface(),
    m_node->get_node_logging_interface()
  );
  auto* item = new PadflieListWidgetItem(id);
  auto* widget = new PadflieWidget(
    nullptr,
    id,
    ros_connection
  );

  m_padflie_widgets[id] = {item, widget, ros_connection};

  connect(this, &ManagerPlugin::availability_message_received,
          widget, [widget, id](int received_id)
  {
      if (received_id == id)
          widget->set_available();
  },
  Qt::QueuedConnection);

  item->setSizeHint(QSize(500, widget->getHeight()));


  m_ui.list_widget->addItem(item);
  m_ui.list_widget->setItemWidget(item, widget);
  m_ui.list_widget->sortItems();
}

void ManagerPlugin::shutdownPlugin()
{
  if (m_update_timer) {
    m_update_timer->cancel();
    m_update_timer.reset();
  }
  m_availability_subscription.reset();
  m_ui.list_widget->clear();
  m_padflie_widgets.clear();
}

void ManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

void ManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}

}  // namespace rqt_padflies

PLUGINLIB_EXPORT_CLASS(rqt_padflies::ManagerPlugin, rqt_gui_cpp::Plugin)
