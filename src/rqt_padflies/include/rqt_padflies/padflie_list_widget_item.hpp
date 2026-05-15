#pragma once

#include "rqt_padflies/padflie_widget.hpp"
#include <QListWidgetItem>

namespace rqt_padflies
{

  class PadflieListWidgetItem : public QListWidgetItem
  {
    public:
      PadflieListWidgetItem(int id)
      : QListWidgetItem()
      , m_id(id)
      , m_message_parser(std::make_shared<PadflieMessageParser>())
      {
      }
      ~PadflieListWidgetItem() = default;

      void set_available() {
        m_message_parser->parse_availability();
      }

      PadflieWidget * get_widget() const {
        return new PadflieWidget(nullptr, m_id, m_message_parser);
      }

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

      std::shared_ptr<PadflieMessageParser> m_message_parser;
    };


}  // namespace rqt_padflies