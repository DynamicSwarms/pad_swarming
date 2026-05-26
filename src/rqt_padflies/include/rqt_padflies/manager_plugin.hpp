#pragma once

#include "rclcpp/rclcpp.hpp"
#include <rqt_gui_cpp/plugin.hpp>

#include "std_msgs/msg/string.hpp"

#include "rqt_padflies/padflie_widget.hpp"
#include "rqt_padflies/padflie_list_widget_item.hpp"

#include "rqt_padflies/padflie_ros_connection.hpp"

#include "ui_manager_plugin.h"

#include <QWidget>
#include <QTimer>
#include <unordered_map>

namespace rqt_padflies
{
struct PadflieListEntry
{
  PadflieListWidgetItem* item;
  PadflieWidget* widget;
  std::shared_ptr<PadflieROSConnection> ros_connection;
};

class ManagerPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
  public:
    ManagerPlugin() {setObjectName("ManagerPlugin");};
    ~ManagerPlugin() {
      if (m_update_timer) {
        m_update_timer->cancel();
        m_update_timer.reset();
      }
    }
    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

  private: 
    void update();
    void m_handle_availability_message(std::shared_ptr<std_msgs::msg::String> msg);
    
    
    void m_signal_handler_availability_message(int id);
    void m_signal_handler_add_padflie(int id);

  protected: 
    Ui::PadfliesManager m_ui;
    QWidget *m_widget;
    
    std::unordered_map<int, PadflieListEntry> m_padflie_widgets;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_availability_subscription;
    std::shared_ptr<rclcpp::TimerBase> m_update_timer;
    std::shared_ptr<rclcpp::Node> m_node;
  signals:
    void availability_message_received(int id);
    void add_padflie(int id);
};
}  // namespace rqt_padflies