#pragma once

#include <QListWidgetItem>

namespace rqt_padflies
{

  class PadflieListWidgetItem : public QListWidgetItem
  {
    public:
      PadflieListWidgetItem(
        int id)
      : QListWidgetItem()
      , m_id(id)
      {}
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