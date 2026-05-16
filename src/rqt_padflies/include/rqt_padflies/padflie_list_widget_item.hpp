#pragma once

#include "rqt_padflies/padflie_widget.hpp"
#include <QListWidgetItem>

namespace rqt_padflies
{

  class PadflieListWidgetItem : public QListWidgetItem, public PadflieWidget
  {
    public:
      PadflieListWidgetItem(
        int id,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface
      )
      : QListWidgetItem()
      , PadflieWidget(nullptr, id, node_topics_interface, node_base_interface, node_graph_interface, node_services_interface)
      , m_id(id)
      {
      }
      ~PadflieListWidgetItem() = default;

      bool 
      operator<(const QListWidgetItem & other) const override {
        const PadflieListWidgetItem * other_widget_item = dynamic_cast<const PadflieListWidgetItem*>(&other);
        if (other_widget_item) {
          return m_id < other_widget_item->m_id;
        } else {
          return QListWidgetItem::operator<(other);
        }
      }
    

    private: 
      int m_id; 
    };


}  // namespace rqt_padflies